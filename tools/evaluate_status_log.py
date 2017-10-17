import argparse
import logging
import matplotlib
matplotlib.use("Agg")
#  matplotlib.rcParams.update({'font.size': 5})
import matplotlib.pyplot as plt
import matplotlib.lines as lines
import os
import re

####################################################################################################

logger = logging.getLogger("Status plotter")
logger.setLevel(logging.INFO)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
formatter = logging.Formatter('[%(levelname)s]%(name)s: %(message)s')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


event_messages = {
        "Init"  : "Initialization OK!",
        "Lost"  : "Tracking LOST!",
        "Reloc" : "Relocalization OK!",
        "Loop"  : "Loop detected!",
        "Reset" : "RESET!",
        "Done"  : "End of process."
        }

####################################################################################################

class OrbSlamSession(object):
    """
    Extracts a Representation of a single ORB SLAM run
    from it's corresponding orb_slam_status.log.
    Holds:
        - all events that happend during the given run
        - total amount of frames in the run
    """
    def __init__(self, path_to_log):
        self.events = self.__create_events(path_to_log)
        self.total_frame_count = self.__read_total_frame_count(self.events[0].log_line)


    def __create_events(self, path_to_log):
        # read log file
        with open(path_to_log, 'r') as file:
            content = file.read()

        # split contents into single lines, filter possibly empty lines
        lines = [line for line in content.splitlines() if len(line) > 0]

        # parse the lines into events
        events = []
        for i, line in enumerate(lines):
            events.append(OrbSlamEvent(line, i))
        return events


    def __read_total_frame_count(self, log_line):
        matches = re.findall(r'(\d+)\/(\d+)', log_line)
        if len(matches) > 0:
            return int(matches[0][1])
        else:
            logger.warning("No total frame count found in line \"{}\"".format(log_line))
            return "UNKNOWN"


    def get_frames_tracked(self):
        """
        Lazy calculates the amount of frames that were successfully tracked.
        """
        if not hasattr(self, "num_frames_tracked"):
            # need to accumulate all frames between init or reloc and Lost, reset or done events
            # as long as there is no Reset or Done in between.
            # Get all 'Init', 'Reloc', 'Lost', 'Reset' and 'Done' events
            desired_events = ('Init', 'Reloc' , 'Lost', 'Reset', 'Done')
            events = (event for event in self.events if event.event_type in desired_events)
            # start counting if event is 'Init' or 'Reloc', stop otherwise
            start = None
            end = None
            tracking_spans = []
            last_event_type = 'Start'
            for event in events:
                logger.debug("Checking event_type: {}".format(event.event_type))
                if event.event_type in ('Init', 'Reloc'):
                    if start is None:
                        start = event.frame_number
                    else:
                        raise Exception("Double start in get_frames_tracked()!")

                if event.event_type in ('Reset', 'Lost'):
                    if last_event_type in ('Reset', 'Lost'):
                        # ignore this, double reset can happen when initialization fails
                        pass
                    elif end is None:
                        # end - 1 because the frame were it's lost wasn't tracked
                        end = event.frame_number - 1
                    else:
                        raise Exception("Double end in get_frames_tracked()!")

                if event.event_type == 'Done':
                    end = event.frame_number - 1
                    if start is not None and end is not None:
                        tracking_spans.append(end-start)
                        start = None
                        end = None
                    break

                if start is not None and end is not None:
                    tracking_spans.append(end-start)
                    start = None
                    end = None

                last_event_type = event.event_type

            # sum the spans up to get the overall tracking count
            self.num_frames_tracked = sum(tracking_spans)
            return self.num_frames_tracked
        else:
            return self.num_frames_tracked


    def get_ratio_tracked(self):
        """
        Returns the ratio of frames that were successfully tracked.
        """
        return self.get_frames_tracked()/float(self.total_frame_count)


    def plot_timeline(self, output_path):
        """
        Plots a timeline showing all events and colouring the stretches inbetween.
        """
        seperator_events = ('Init', 'Lost', 'Reset', 'Reloc', 'Done')

        def cm2inch(*tupl):
            inch = 2.54
            if isinstance(tupl[0], tuple):
                return tuple(i/inch for i in tupl[0])
            else:
                return tuple(i/inch for i in tupl)

        fig = plt.figure(figsize=cm2inch(20, 0.5))
        ax = fig.add_subplot(111)
        ax.yaxis.set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.xaxis.set_ticks_position('bottom')
        ax.set_xlim([0, self.total_frame_count])
        #  ax.set_aspect(100)
        ax.set_xlabel("Frame")

        start = 0
        last_seperator_event_type = 'Start'
        for event in self.events:
            if event.event_type in seperator_events:
                end = event.frame_number
                x = range(start, end)
                logger.debug("Filling between {} and {}".format(x[0], x[-1]))
                if last_seperator_event_type in ('Init', 'Reloc'):
                    # tracking
                    color = 'green'
                elif last_seperator_event_type == 'Lost':
                    # relocalizing
                    color = 'red'
                elif last_seperator_event_type in ('Start', 'Reset'):
                    # initializing
                    color = 'yellow'
                logger.debug("With color {}".format(color))

                ax.fill_between(x, 0, 1, color=color)
                #  line = lines.Line2D([end-5, end-5], [0, 1], linewidth=0.05, color = 'black')
                #  ax.add_line(line)

                start = end
                if event.event_type == 'Lost':
                    start = start - 1
                last_seperator_event_type = event.event_type

            # draw an R for a reset
            if event.event_type == 'Reset':
                ax.text(event.frame_number, 1.2, "R", size='xx-small', horizontalalignment='center')
            # draw an L and a blue line for a Loop
            elif event.event_type == 'Loop':
                ax.text(event.frame_number, 1.2, "L", size='xx-small', horizontalalignment='center')
                line = lines.Line2D([event.frame_number, event.frame_number], [0, 1], linewidth=0.2,
                                    color = 'blue')
                ax.add_line(line)

        plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
        logger.info("Plot saved to: {}".format(output_path))


####################################################################################################

class OrbSlamEvent(object):
    """
    Matches a line from orb_slam_status.log
    to a given event defined by "event_messages"
    Holds:
        - the events frame number
        - the type of event
    """
    def __init__(self, log_line, event_number):
        self.log_line = log_line
        self.frame_number = self.__extract_frame_number(log_line)
        self.event_number = event_number
        self.event_type = self.__extract_event_type(log_line)


    def __extract_event_type(self, log_line):
        # match log line with event messages
        for event, message in event_messages.items():
            matches = re.findall(message, log_line)
            if len(matches) > 0:
                logger.debug(
                        "Event {}: {} at frame {}".format(self.event_number, event, self.frame_number))
                return event

        logger.warning("Line without matching event found!")
        return "BROKEN_EVENT"


    def __extract_frame_number(self, log_line):
        # store the frame number of the event
        matches = re.findall(r'(\d+)\/(\d+)', log_line)
        if len(matches) > 0:
            return int(matches[0][0])
        else:
            logger.warning(
                    "Frame number of line could not be extracted from line {}".format(log_line))
            return "UNKNOWN"



####################################################################################################

def main():
    # parse command line arguments
    arg_parser = argparse.ArgumentParser(description="Status plotter plots a timeline-like"
                                         + " plot which shows when which events (loops found etc.)"
                                         + " happend during a session.",
                                         formatter_class=argparse.RawTextHelpFormatter)
    arg_parser.add_argument("input_path", help="Path to orb_slam_status.log")
    args = arg_parser.parse_args()

    input_path = args.input_path

    # check if input is a file
    if not os.path.isfile(input_path):
        print("Input needs to be a log file. \"{}\" is not a file.".format(input_path))

    # create OrbSlamSession object which interprets the log
    session = OrbSlamSession(input_path)
    logger.info("Amount frames tracked: {} that's {}%".format(session.get_frames_tracked()
                                                             ,session.get_ratio_tracked()*100))
    session.plot_timeline("status_line.pdf")

if __name__ == '__main__':
    main()


