import cv2 as cv
from abc import ABC, abstractmethod
from argparse import ArgumentParser
from os import listdir
from os.path import splitext, join


def _file_extension(filepath):
    return splitext(filepath)[1]


def _is_png(filepath):
    return _file_extension(filepath) == '.png'


def test_command(args):
    image_provider = ImageFromFileProvider(args.color, args.depth)

    while image_provider.has_frames():
        frames = image_provider.next_frames()

        cv.imshow('rgb', frames['rgb'])
        cv.waitKey(30)


def prepare_args_parser():
    parser = ArgumentParser(description='Modular SLAM research module for Visual Odometry algorithms')
    subparsers = parser.add_subparsers(help='')

    test_subparser = subparsers.add_parser('test', help='a help')
    test_subparser.add_argument('-c', '--color', required=True, help='Directory of RGB images')
    test_subparser.add_argument('-d', '--depth', required=True, help='Directory of depth images')
    test_subparser.set_defaults(handle_command=test_command)

    return parser


class ImageProvider(ABC):
    @abstractmethod
    def next_frames(self):
        return {'rgb': None, 'depth': None}

    @abstractmethod
    def has_frames(self):
        return False


class ImageFromFileProvider(ImageProvider):
    def __init__(self, rgb_path, depth_path=None):
        self._init_paths(rgb_path, depth_path)
        self.current_index = 0

    def _init_paths(self, rgb_path, depth_path):
        self._rgb_paths = sorted([join(rgb_path, f) for f in listdir(rgb_path) if _is_png(f)])
        self._depth_paths = sorted([join(depth_path, f) for f in listdir(depth_path) if _is_png(f)])

        import pprint
        pprint.pprint(self._depth_paths)

        assert len(self._rgb_paths) == len(self._depth_paths)

    def _read_depth(self, path):
        img = cv.imread(path, cv.IMREAD_ANYDEPTH)
        return img

    def _read_rgb(self, path):
        img = cv.imread(path)
        return img

    def next_frames(self):
        rgb_path = self._rgb_paths[self.current_index]
        depth_path = self._depth_paths[self.current_index]
        frames = {
            'rgb': self._read_rgb(rgb_path),
            'depth': self._read_depth(depth_path)
        }
        self.current_index += 1

        return frames

    def has_frames(self):
        return self.current_index < len(self._rgb_paths)


def main():
    parser = prepare_args_parser()
    args = parser.parse_args()

    args.handle_command(args)


if __name__ == '__main__':
    main()
