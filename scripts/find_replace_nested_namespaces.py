#!/usr/bin/env python3

"""
Current script replaces instances of multiple namespaces with the modern
C++17-equivalent nested namespaces

.. seealso::
* `<https://github.com/MRPT/mrpt/issues/752>_`
* `<http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2014/n4230.html>_`

"""

import click
import sh
import re
from typing import List


@click.command()
@click.option('-r', '--root-dir', help='Top-level directory to operate in',
              required=True, type=click.Path(exists=True))
@click.option('-e', '--extensions', help='Extensions which are to be considered', multiple=True)
def main(root_dir: str, extensions: List[str]):
    # list of files to operate in
    fpaths = []  # type: List[str]
    # files that need to be checked manually
    files_manual_check = []  # type: List[str]
    num_files_modified = 0  # type: int

    # find all the files we're interested in
    for e in extensions:
        fpaths.extend([i.rstrip() for i in sh.find(root_dir, "-iname",
                                                   "*.{}".format(e))])

    expr_text = "namespace (\w*)[\s]{"
    expr = re.compile(expr_text)
    # we now have a list of files
    for fpath in fpaths:
        with open(fpath, 'rU') as f:
            old_conts = f.read()
        # do we have more than a single match
        ns_expr_results = expr.findall(old_conts)
        times_found = len(ns_expr_results)
        if times_found > 1:
            start_str = "\s*".join([expr_text] * times_found)
            # match whitespace characters or comments after '}'
            ending_str_after = "(\s*)(//.*)?(\s*)"
            end_str = ending_str_after.join(['}'] * times_found)
            end_str += ending_str_after

            valid_start_results = re.search(start_str, old_conts)
            valid_end_results = re.search(end_str, old_conts)

            if not valid_start_results or not valid_end_results:
                files_manual_check.append(fpath)
                continue
            # compose new contents
            new_conts = old_conts

            # convert the start
            # grab the namespace names
            ns_names = [g for g in valid_start_results.groups()]
            # compose the new start
            new_start = " ".join(["namespace", "::".join(ns_names), ])
            new_start = "\n".join([new_start, "{"])

            new_conts = "".join([new_conts[:valid_start_results.start()],
                                 new_start,
                                 new_conts[valid_start_results.end():],
                                 "\n"])

            # need to update the end characters position
            valid_end_results = list(re.finditer(end_str, new_conts))[-1]
            # convert the end
            # convert only the count of instances that you are
            # interested in, starting from the bottom
            new_end = "}"
            new_conts = "".join([new_conts[:valid_end_results.start()],
                                 new_end,
                                 "\n",
                                 new_conts[valid_end_results.end():],
                                 "\n"])

            # flush the new contents
            with open(fpath, 'w') as f:
                num_files_modified += 1
                f.write(new_conts)


    print("{} files modified".format(num_files_modified))
    print("{} files to check manually:\n{}".format(
        len(files_manual_check),
        '\n'.join(["\t{}".format(i) for i in
                   files_manual_check])))

if __name__ == "__main__":
    main()
