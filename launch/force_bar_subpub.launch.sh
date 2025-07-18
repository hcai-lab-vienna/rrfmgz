#!/bin/bash

# for i in {0..4}; do python3 src/force_bar_subpub.py $i & done

python3 src/force_bar_subpub.py 0 &
python3 src/force_bar_subpub.py 1 &
python3 src/force_bar_subpub.py 2 &
python3 src/force_bar_subpub.py 3 &
python3 src/force_bar_subpub.py 4
kill $(jobs -p)