#! /usr/bin/env python3

import rclpy


def main():
    rclpy.init()

    print('The shelf transport application has started.')

    rclpy.shutdown()


if __name__ == '__main__':
    main()