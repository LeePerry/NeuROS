import argparse
import os
import sys

class CommandLineInterface(argparse.ArgumentParser):

    @classmethod
    def from_command_line(cls):
        parser = cls(
            formatter_class=argparse.RawTextHelpFormatter,
            description="NeuROS\n\n" +
                "An Integration Framework for Heterogenous Systems Neuroscience")

        parser.add_argument("-p",
                            "--project_path",
                            type=str,
                            required=True,
                            help="The path to the project configuration json file")
        parser.add_argument("-n",
                            "--node",
                            type=str,
                            required=False,
                            action="append",
                            help="[OPTIONAL] " +
                                "The name of a node as specified in the project configuration. " +
                                "Can be specified multiple times to launch a subset. "+
                                "Default is to launch all nodes.")
        args = parser.parse_args()

        if not os.path.isfile(args.project_path):
            parser.print_help()
            print(f"\nProject path '{args.project_path}' is not a file!\n")
            sys.exit(1)

        if args.node:
            args.node = set(args.node)

        return args

    def error(self, message):
        self.print_help()
        print(f"\n{message}!\n")
        sys.exit(1)
