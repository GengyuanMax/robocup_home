#!/usr/bin/env python

import sys
import os.path
import csv

# This is a tiny script to help you creating a CSV file from a face
# database with a similar hierarchie:
#
#  philipp@mango:~/facerec/data/at$ tree
#  .
#  |-- README
#  |-- s1
#  |   |-- 1.pgm
#  |-- s2
#  |   |-- 1.pgm
#  ...
#  |-- s40
#  |   |-- 1.pgm
#

if __name__ == "__main__":

    if len(sys.argv) != 2:
        print("usage: create_csv <base_path>")
        sys.exit(1)

    BASE_PATH=sys.argv[1]
    SEPARATOR=";"

    database_file = BASE_PATH + "/database.csv"
    print(database_file)

    with open(database_file, "w") as f:
        writer = csv.writer(f)
        content = []

        for dirname, dirnames, filenames in os.walk(BASE_PATH):
            dirname = os.path.abspath(dirname)
            for subdirname in dirnames:
                label = subdirname
                subject_path = os.path.join(dirname, subdirname)
                for filename in os.listdir(subject_path):
                    abs_path = "{}/{}".format(subject_path, filename)
                    print("{}{}{}".format(abs_path, SEPARATOR, label))
                    content.append([abs_path, label])
                    
        writer.writerows(content)
