import sys

def parser(log):
    with open(log, "r") as f:
        lines = f.readlines()
        if lines == "":
            return True
        else:
            return False

log = str(sys.argv[1])

parser(log)