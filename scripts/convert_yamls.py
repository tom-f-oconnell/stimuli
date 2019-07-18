#!/usr/bin/env python3

import os
import glob
import shutil
import sys

import yaml


data_root = '/mnt/nas/Kristina/Behaviordata/ROSvideosoriginal'

from_f1 = glob.glob(data_root + '/*/*_stimuli.yaml')
from_f2 = glob.glob(data_root + '/*/*/*_stimuli.yaml')

yaml_backup_dir = 'stimuli_yaml_backups'
abs_yaml_backup_dir = os.path.join(data_root, yaml_backup_dir)
if not os.path.exists(abs_yaml_backup_dir):
    os.mkdir(abs_yaml_backup_dir)

for orig_yaml in from_f1 + from_f2:
    # don't process existing backups
    if os.path.dirname(orig_yaml) == abs_yaml_backup_dir:
        continue

    # backup original file
    shutil.copyfile(orig_yaml,
        os.path.join(abs_yaml_backup_dir, os.path.basename(orig_yaml)))

    with open(orig_yaml, 'r') as f:
        data_dict = yaml.load(f)

    already_converted = False
    for s in ('left', 'right'):
        try:
            odors2pins = data_dict.pop('odors2{}_pins'.format(s))

        except KeyError:
            already_converted = True
            break

        # inverting the dictionary, and converting tuple to standard list type,
        # so that the dict can be represented in a wider selection of yaml
        # implementations
        pins2odors = dict()
        for o, p in odors2pins.items():
            pins2odors[p] = list(o)

        data_dict['{}_pins2odors'.format(s)] = pins2odors

    if not already_converted:
        print('converting {}'.format(orig_yaml))
        with open(orig_yaml, 'w') as f:
            yaml.dump(data_dict, f)

