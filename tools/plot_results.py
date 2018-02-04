import argparse
import json
import logging
import matplotlib
matplotlib.use("Qt4Agg")
matplotlib.rcParams.update({'font.size': 15})
import matplotlib.pyplot as plt
import matplotlib.lines as lines
from matplotlib.backends.backend_agg import FigureCanvasAgg
import os
import re
from evaluate_status_log import OrbSlamSession
from matplotlib import gridspec
from PIL import Image
import numpy as np

####################################################################################################

logger = logging.getLogger("Results plotter")
logger.setLevel(logging.INFO)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
formatter = logging.Formatter('[%(levelname)s]%(name)s: %(message)s')
console_handler.setFormatter(formatter)
logger.addHandler(console_handler)


####################################################################################################
def plot_results(session, results_json, output_path, plot_frame_label):
    """
    Plots a timeline showing all events and colouring the stretches inbetween.
    This also adds boxplots for the results in results_json.
    """

    seperator_events = ('Update', 'Init', 'Lost', 'Reset', 'Reloc', 'Done')

    def cm2inch(*tupl):
        inch = 2.54
        if isinstance(tupl[0], tuple):
            return tuple(i/inch for i in tupl[0])
        else:
            return tuple(i/inch for i in tupl)

    fig = plt.figure(figsize=cm2inch(20, 0.5))
    gs = gridspec.GridSpec(1, 3, width_ratios=[12, 1, 1])
    ax = plt.subplot(gs[0])
    ax.yaxis.set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.xaxis.set_ticks_position('bottom')
    ax.set_xlim([0, session.total_frame_count])
    ax.minorticks_on()
    if plot_frame_label:
        ax.set_xlabel("Frame")

    start = 0
    last_seperator_event_type = 'Start'
    for event in session.events:
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

    with open(results_json, 'r') as file:
        content = json.load(file)

    flierprops = dict(marker='.', markersize=1, linestyle='none')

    ax1 = plt.subplot(gs[1])
    ax1.spines['right'].set_visible(False)
    ax1.spines['top'].set_visible(False)
    ax1.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom='off',      # ticks along the bottom edge are off
        top='off')         # ticks along the top edge are off
    boxes = ax1.boxplot(content["tracked_ratios"], labels=["Tracked"], widths=(0.6,),
            flierprops=flierprops)
    lims = [max(0,int(min(content["tracked_ratios"]) - 1)), int(max(content["tracked_ratios"]) + 1)]
    ax1.set_yticks(lims)
    ax1.set_ylim(lims)

    times_lost_per_thousand = []
    for times_lost in content["times_lost"]:
        times_lost_per_thousand.append((times_lost/session.total_frame_count)*1000)
    ax2 = plt.subplot(gs[2])
    ax2.spines['right'].set_visible(False)
    ax2.spines['top'].set_visible(False)
    ax2.tick_params(
        axis='x',          # changes apply to the x-axis
        which='both',      # both major and minor ticks are affected
        bottom='off',      # ticks along the bottom edge are off
        top='off')         # ticks along the top edge are off
    boxes = ax2.boxplot(times_lost_per_thousand, labels=["Lost"], widths=(0.6, ),
            flierprops=flierprops)
    lims = [int(max(0, min(times_lost_per_thousand) - 1)), int(max(times_lost_per_thousand) + 1)]
    ax2.set_yticks(lims)
    ax2.set_ylim(lims)


    plt.savefig(output_path, bbox_inches='tight', pad_inches=0)
    logger.info("Plot saved to: {}".format(output_path))


def plot_continous(final_log, plot_frame_label):
    """
    Plots a timeline showing all events and colouring the stretches inbetween.
    This function continously updates the output plot.
    """

    seperator_events = ('Update', 'Init', 'Lost', 'Reset', 'Reloc', 'Done')

    def cm2inch(*tupl):
        inch = 2.54
        if isinstance(tupl[0], tuple):
            return tuple(i/inch for i in tupl[0])
        else:
            return tuple(i/inch for i in tupl)

    session = OrbSlamSession(final_log)

    fig = plt.figure(figsize=cm2inch(20, 0.1))
    #  fig.patch.set_facecolor('black')
    ax = fig.add_subplot(1, 1, 1)
    ax.yaxis.set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['left'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.xaxis.set_ticks_position('bottom')
    ax.set_xlim([0, session.total_frame_count])
    ax.minorticks_on()

    if plot_frame_label:
        ax.set_xlabel("Frame")

    plt.ion()

    start = 0
    last_seperator_event_type = 'Start'
    last_events = []
    new_events = []
    while True:
        if len(last_events) != session.events:
            new_events = session.events[len(last_events):]
        for event in new_events:
            if event.event_type in seperator_events:
                end = event.frame_number
                #  logger.debug("Filling between {} and {}".format(x[0], x[-1]))
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

                #  ax.fill_between(x, 0, 1, color=color)
                ax.axvspan(start, end, ymin=0, ymax=0.8, alpha=1, color=color)
                #  line = lines.Line2D([end-5, end-5], [0, 1], linewidth=0.05, color = 'black')
                #  ax.add_line(line)

                start = end
                if event.event_type == 'Lost':
                    start = start - 1
                last_seperator_event_type = event.event_type

            # draw an R for a reset
            if event.event_type == 'Reset':
                ax.text(event.frame_number, 0.9, "R", size='xx-small', horizontalalignment='center')
            # draw an L and a blue line for a Loop
            elif event.event_type == 'Loop':
                ax.text(event.frame_number, 0.9, "L", size='small', horizontalalignment='center')
                line = lines.Line2D([event.frame_number, event.frame_number], [0, 0.8], linewidth=0.4,
                                    color = 'blue')
                ax.add_line(line)

        plt.pause(0.05)

        last_events = session.events
        new_events = []
        session = OrbSlamSession(final_log)


def main():
    # parse command line arguments
    arg_parser = argparse.ArgumentParser(description="Status plotter plots a timeline-like"
                                         + " plot which shows when which events (loops found etc.)"
                                         + " happend during a session.",
                                         formatter_class=argparse.RawTextHelpFormatter)
    arg_parser.add_argument("results_json", help="Path to results json")
    arg_parser.add_argument("final_log", help="Path to log used for final plot")
    arg_parser.add_argument('-l', '--label', action='store_true', help="Plots the \"frame\" label",
                            default=False)
    arg_parser.add_argument('-c', '--continous', action='store_true',
                            help="Continously updates plot", default=False)
    args = arg_parser.parse_args()

    results_json = args.results_json
    final_log    = args.final_log

    # create the final session
    session = OrbSlamSession(final_log)

    if args.continous:
        plot_continous(final_log, args.label)
    else:
        plot_results(session, results_json, "final_plot.pdf", args.label)


if __name__ == '__main__':
    main()
