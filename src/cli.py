import argparse
import os
import sys

def parse_cli_args():

    parser = argparse.ArgumentParser(
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
                        help="The name of a node as specified in the project " +
                             "configuration (multiple allowed, defaults to all)")
    args = parser.parse_args()

    if not os.path.isfile(args.project_path):
        parser.print_help()
        print("\n\nPlease provide a valid project path!\n\n")
        sys.exit(1)

    return args
