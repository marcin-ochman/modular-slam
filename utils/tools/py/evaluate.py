#!/usr/bin/env python3

import argparse
import sys
import os
import logging
import docker
import re
import pandas as pd

from evo.tools import file_interface, plot
from evo.core import sync, metrics
from evo.tools.settings import SETTINGS
from associate import read_file_list, associate
from matplotlib import pyplot as plt


ORB_SLAM_3_TAG = 'modular_slam/orbslam3'
STELLA_VSLAM_TAG = 'modular_slam/stellavslam'

def prepare_trajectories(trajectories, ref_trajectory_path,
                         names=[]):

    traj_ref = file_interface.read_tum_trajectory_file(ref_trajectory_path)
    trajectories_dict = {}
    use_names = len(names) == len(trajectories)
    for idx, trajectory_file in enumerate(trajectories):
        traj_est = file_interface.read_tum_trajectory_file(trajectory_file)
        traj_ref_associated, traj_est = sync.associate_trajectories(traj_ref, traj_est)

        traj_est.align_origin(traj_ref_associated)
        name = names[idx] if use_names else trajectory_file
        trajectories_dict[name] = traj_est

    return trajectories_dict, traj_ref


def generate_plots(trajectories, ref_trajectory, output_dir, names=[]):
    trajectories, ref_trajectory = prepare_trajectories(trajectories, ref_trajectory)

    plot_mode = plot.PlotMode.xyz
    fig_traj = plt.figure()
    ax_traj = plot.prepare_axis(fig_traj, plot_mode)

    plot.traj(ax_traj, plot_mode, ref_trajectory,
              style=SETTINGS.plot_reference_linestyle,
              color=SETTINGS.plot_reference_color,
              label='Groundtruth',
              alpha=SETTINGS.plot_reference_alpha)
    plot.draw_coordinate_axes(ax_traj, ref_trajectory, plot_mode,
                              SETTINGS.plot_reference_axis_marker_scale)

    fig_xyz, axarr_xyz = plt.subplots(3, sharex='col', figsize=tuple(SETTINGS.plot_figsize))
    fig_rpy, axarr_rpy = plt.subplots(3, sharex='col', figsize=tuple(SETTINGS.plot_figsize))
    start_time = ref_trajectory.timestamps[0]

    plot.traj_xyz(
        axarr_xyz, ref_trajectory, style=SETTINGS.plot_reference_linestyle,
        color=SETTINGS.plot_reference_color, label='Groundtruth',
        alpha=SETTINGS.plot_reference_alpha,
        start_timestamp=start_time)
    plot.traj_rpy(
        axarr_rpy, ref_trajectory, style=SETTINGS.plot_reference_linestyle,
        color=SETTINGS.plot_reference_color, label='Groundtruth',
        alpha=SETTINGS.plot_reference_alpha,
        start_timestamp=start_time)

    for idx, (name, trajectory) in enumerate(trajectories.items()):
        color = next(ax_traj._get_lines.prop_cycler)['color']
        plot.traj_xyz(axarr_xyz, trajectory, SETTINGS.plot_trajectory_linestyle,
                      color, names[idx],
                      alpha=SETTINGS.plot_trajectory_alpha,
                      start_timestamp=start_time)

        plot.traj_rpy(axarr_rpy, trajectory, SETTINGS.plot_trajectory_linestyle,
                      color, names[idx],
                      alpha=SETTINGS.plot_trajectory_alpha,
                      start_timestamp=start_time)


        plot.traj(ax_traj, plot_mode, trajectory,
                  SETTINGS.plot_trajectory_linestyle, color,
                  names[idx], alpha=SETTINGS.plot_trajectory_alpha)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    fig_xyz.savefig(os.path.join(output_dir, 'xyz.eps'))
    fig_rpy.savefig(os.path.join(output_dir, 'rpy.eps'))
    fig_traj.savefig(os.path.join(output_dir, 'traj3d.eps'))

    plt.show()


def generate_plots_command(args):
    generate_plots(args.trajectories, args.ref_trajectory, args.output, args.names)


def generate_data_command(args):
    trajectories, ref_trajectory = prepare_trajectories(args.trajectories, args.ref_trajectory, args.names)
    results = {}
    ape_metric = metrics.APE(metrics.PoseRelation.full_transformation)

    for name, trajectory in trajectories.items():
        traj_ref_associated, traj_est = sync.associate_trajectories(ref_trajectory, trajectory)
        ape_metric.process_data((traj_ref_associated, traj_est))
        ape_stats = ape_metric.get_all_statistics()
        results[name] = ape_stats

    results_dict = { key: [metrics[metric_name] for metric_name in sorted(metrics.keys()) ]
                                  for key, metrics in results.items()}
    columns = list(results[list(results.keys())[0]].keys())
    df = pd.DataFrame.from_dict(results_dict,
                                orient='index', columns=columns)

    output_csv_path = os.path.join(args.output, 'results.csv')
    logging.info(f'Saved CSV results to {output_csv_path}')
    df.to_csv(output_csv_path)


def generate_results(args):
    if args.orbslam3:
        run_orbslam3(args.tum_dataset_dir, args.output)

    if args.stellavslam:
        run_stella(args.tum_dataset_dir, args.output)


def generate_association_file(sequence_dir):
    rgb_list = read_file_list(os.path.join(sequence_dir, 'rgb.txt'))
    depth_list = read_file_list(os.path.join(sequence_dir, 'depth.txt'))
    matches = associate(rgb_list, depth_list, 0.0, 0.02)
    rgbd_output_path = os.path.join(sequence_dir, 'rgbd.txt')
    with open(rgbd_output_path, 'w') as f:
        for a,b in matches:
            f.write('{:.6f} {} {} {}\n'.format(a,' '.join(rgb_list[a]), b, ' '.join(depth_list[b])))



def run_slam_in_docker(client, image, commands, tum_dataset_dir, results_dir):
    mounted_volumes = {
        tum_dataset_dir: {'bind': '/dataset/tum/', 'mode': 'rw'},
        '/tmp/.X11-unix/': {'bind':'/tmp/.X11-unix', 'mode': 'ro'},
        results_dir: {'bind':'/result/', 'mode': 'rw'}
    }

    envs = {'DISPLAY': os.getenv('DISPLAY')}
    logging.info(f'Running {image} with TUM dataset {tum_dataset_dir}')
    container = client.containers.run(image, command='/bin/bash',
                                      volumes=mounted_volumes, environment=envs, tty=True, detach=True,
                                      device_requests=[
                                          docker.types.DeviceRequest(count=-1, capabilities=[['gpu']])
                                      ])

    for command in commands:
        result = container.exec_run(command)

    container.stop()
    container.remove()


def run_orbslam3(tum_dataset_dir, output_dir):
    client = docker.from_env()

    if not has_orbslam3_img(client):
        logging.info(f'No {ORB_SLAM_3_TAG} image detected. Building docker image. It may take a while.')
        build_orbslam3_image(client)
    else:
        logging.info('No need to build ORBSLAM3 image')

    if tum_dataset_dir is not None:
        tum_sequences_dirs = [os.path.join(tum_dataset_dir, f) for f in os.listdir(tum_dataset_dir)
                              if os.path.isdir(os.path.join(tum_dataset_dir, f))]

        for sequence_dir in tum_sequences_dirs:
            if 'rgbd,txt' not in os.listdir(sequence_dir):
                logging.info(f'No association file found in {sequence_dir}. Generating rgbd.txt association file')
                generate_association_file(sequence_dir)

            sequence_name = os.path.basename(sequence_dir)
            sequence_id = re.findall(r'\d+', sequence_name)[0]

            dataset_directory_in_container = os.path.join('/dataset/tum', sequence_name)
            rgbd_path_in_container = os.path.join(dataset_directory_in_container, 'rgbd.txt')
            command_slam = f'/tmp/ORB_SLAM3/Examples/RGB-D/rgbd_tum /tmp/ORB_SLAM3/Vocabulary/ORBvoc.txt ' + \
                f'/tmp/ORB_SLAM3/Examples/RGB-D/TUM{sequence_id}.yaml ' + \
                f'{dataset_directory_in_container} {rgbd_path_in_container}'
            command_copy = f'cp CameraTrajectory.txt /result/{sequence_name}.txt'
            results_dir = os.path.join(output_dir, 'orbslam3', 'tum')

            if not os.path.exists(results_dir):
                os.makedirs(results_dir)

            run_slam_in_docker(client, ORB_SLAM_3_TAG, [command_slam, command_copy],
                               tum_dataset_dir, results_dir)

def run_stella(tum_dataset_dir, output_dir):
  client = docker.from_env()

  if not has_stellavslam_img(client):
      logging.info(f'No {STELLA_VSLAM_TAG} image detected. Building docker image. It may take a while.')
      build_stellavslam_image(client)
  else:
      logging.info('No need to build stella vslam image')

  if tum_dataset_dir is not None:
      tum_sequences_dirs = [os.path.join(tum_dataset_dir, f) for f in os.listdir(tum_dataset_dir)
                            if os.path.isdir(os.path.join(tum_dataset_dir, f))]

      for sequence_dir in tum_sequences_dirs:
          if 'rgbd,txt' not in os.listdir(sequence_dir):
              logging.info(f'No association file found in {sequence_dir}. Generating rgbd.txt association file')
              generate_association_file(sequence_dir)

          sequence_name = os.path.basename(sequence_dir)
          sequence_id = re.findall(r'\d+', sequence_name)[0]

          dataset_directory_in_container = os.path.join('/dataset/tum', sequence_name)
          rgbd_path_in_container = os.path.join(dataset_directory_in_container, 'rgbd.txt')
          command_slam = f'./run_tum_rgbd_slam -v orb_vocab.fbow  -d {dataset_directory_in_container}' + \
              f' -c ../example/tum_rgbd/TUM_RGBD_rgbd_{sequence_id}.yaml --no-sleep ' + \
              f' --auto-term --map-db-out map.msg --eval-log-dir ./'
          command_copy = f'cp frame_trajectory.txt /result/{sequence_name}.txt'

          results_dir = os.path.join(output_dir, 'stella', 'tum')

          if not os.path.exists(results_dir):
              os.makedirs(results_dir)

          run_slam_in_docker(client, STELLA_VSLAM_TAG, [command_slam, command_copy],
                             tum_dataset_dir, results_dir)

def parse_args():
    parser = argparse.ArgumentParser()

    subparsers = parser.add_subparsers(title='subcommands',
                                       description='Valid subcommands',
                                       help='Additional help')

    parent_parser = argparse.ArgumentParser(add_help=False)
    parent_parser.add_argument('--ref_trajectory', help='Path to file containing trajectory in TUM format',
                               type=str, required=True)
    parent_parser.add_argument('--trajectories', help='Paths to trajectories in TUM format', type=str, nargs='+')
    parent_parser.add_argument('--names', help='Names of evaluated algorithms', type=str, nargs='+')
    parent_parser.add_argument('--output', help='', type=str)

    plot_subparser = subparsers.add_parser('plot', help='Command for generating plots', parents=[parent_parser])
    plot_subparser.set_defaults(command=generate_plots_command)

    data_subparser = subparsers.add_parser('data', help='Command for generating data results including tables, csv etc.',
                                           parents=[parent_parser])
    data_subparser.set_defaults(command=generate_data_command)

    generate_subparser = subparsers.add_parser('generate', help='Command for generating all results')
    generate_subparser.set_defaults(command=generate_results)
    generate_subparser.add_argument('--stellavslam', help='Adds StellaVSLAM to compare', action='store_true')
    generate_subparser.add_argument('--orbslam3', help='Adds ORBSLAM3 to compare', action='store_true')
    generate_subparser.add_argument('--tum_dataset_dir', help='Dataset directory for TUM extracted datasets containing ' + \
                                    ' e.g. rgbd_dataset_freiburg1_desk, rgbd_dataset_freiburg2_xyz etc',
                                    default=None)
    generate_subparser.add_argument('--output', help='', default=os.path.abspath('./results'))

    return parser.parse_args()


def build_orbslam3_image(client):
    kwargs = {'path': os.path.join(os.path.dirname(__file__), '3rdparty_slam', 'orb_slam_3'),
              'tag': ORB_SLAM_3_TAG,
              'dockerfile': 'Dockerfile'}
    build_image(client, **kwargs)

def build_stellavslam_image(client):
    kwargs = {'path': os.path.join(os.path.dirname(__file__), '3rdparty_slam', 'stella_vslam'),
              'tag': STELLA_VSLAM_TAG,
              'dockerfile': 'Dockerfile'}
    build_image(client, **kwargs)


def build_image(client: docker.DockerClient, **kwargs):
    logging.info(f'Builing {kwargs["tag"]} image. It may take a while.')
    client.images.build(**kwargs)

def has_orbslam3_img(client: docker.DockerClient) -> bool:
    return has_docker_img(client, ORB_SLAM_3_TAG)

def has_stellavslam_img(client: docker.DockerClient) -> bool:
    return has_docker_img(client, STELLA_VSLAM_TAG)

def has_docker_img(client: docker.DockerClient, img_name: str) -> bool:
    images = client.images.list()

    return any(img_name in img_tag
               for img in images
               for img_tag in img.tags)

if __name__ == '__main__':
    logging.basicConfig(encoding='utf-8', level=logging.INFO)

    args = parse_args()
    args.command(args)
