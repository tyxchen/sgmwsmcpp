#!/usr/bin/env python3

"""
Compares the accuracy of run results to source data
"""

import os
import csv
from pathlib import Path
import argparse


def main():
    parser = argparse.ArgumentParser(description="Compare the accuracy of run results to source data.")
    parser.add_argument("reference_dir", help="Directory of source data")
    parser.add_argument("results_dir", help="Directory of run results")
    parser.add_argument("-v", "--verbose", action="store_true", help="Be verbose")

    args = parser.parse_args()

    reference_dir = Path(args.reference_dir)
    test_dir = Path(args.results_dir)
    verbose = args.verbose

    reference_files = sorted(reference_dir.glob("*.csv"))
    test_results = sorted(test_dir.glob("*.csv"))

    if len(reference_files) != len(test_results):
        print(reference_files)
        print(test_results)
        raise ValueError("Reference and test result files do not match")

    total_correct_matches = 0
    total_matches = 0

    for i in range(len(test_results)):
        ref_data_file = open(reference_files[i], newline='')
        test_result_file = open(test_results[i])
        ref_data = set()
        test_result = set()
        err = None

        try:
            edge_builder = set()
            last_matching = -1
            ref_reader = csv.DictReader(ref_data_file)
            for row in ref_reader:
                pidx = row["surface"]
                idx = row["idx"]
                matching = row["matching"]
                if matching == last_matching or last_matching == -1:
                    edge_builder.add((pidx, idx))
                else:
                    ref_data.add(frozenset(edge_builder))
                    edge_builder = set()
                last_matching = matching
            ref_data.add(frozenset(edge_builder))

            last_matching = -1
            for line in test_result_file:
                pidx, idx, matching = map(lambda s: s.strip(), line.split(","))
                if matching == last_matching or last_matching == -1:
                    edge_builder.add((pidx, idx))
                else:
                    test_result.add(frozenset(edge_builder))
                    edge_builder = set()
                last_matching = matching
            test_result.add(frozenset(edge_builder))
        except Exception as e:
            err = e
        finally:
            ref_data_file.close()
            test_result_file.close()
            if err is not None: raise err

        print(os.path.basename(reference_files[i]))
        if ref_data != test_result and verbose:
            print(f"Unique to {reference_files[i]}:")
            for edge in (ref_data - test_result):
                print("  {{{}}}".format(", ".join([f"({p}, {i})" for (p, i) in edge])))
            print(f"Unique to {test_results[i]}:")
            for edge in (test_result - ref_data):
                print("  {{{}}}".format(", ".join([f"({p}, {i})" for (p, i) in edge])))

        total_correct_matches += len(ref_data & test_result)
        total_matches += len(ref_data)
        print("Match rate: {:.2f}%".format(len(ref_data & test_result) / len(ref_data) * 100))
        print("---------")

    print("Total match rate: {:.2f}%".format(total_correct_matches / total_matches * 100))

if __name__ == "__main__":
    main()
