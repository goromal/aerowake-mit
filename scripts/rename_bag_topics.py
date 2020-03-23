#!/usr/bin/python

from rosbag import Bag
import sys, os

FULLBAGPATH = sys.argv[1]
BAGNAME = os.path.basename(FULLBAGPATH).split('.')[0]
BAGDNAME = os.path.dirname(FULLBAGPATH)

replacements = dict()

print 'Remappings for %s.bag:' % BAGNAME
for i in range(0, len(sys.argv[2:])/2):
    idx_from = 2 + 2*i + 0
    idx_to   = 2 + 2*i + 1
    print '%s -> %s' % (sys.argv[idx_from], sys.argv[idx_to])
    replacements[sys.argv[idx_from]] = sys.argv[idx_to]

print 'Writing to %s-RENAMED.bag...' % BAGNAME
with Bag(os.path.join(BAGDNAME, BAGNAME + '-RENAMED.bag'), 'w') as out_bag:
    for topic, msg, t in Bag(FULLBAGPATH):
        if topic in replacements:
            out_bag.write(replacements[topic], msg, t)
        else:
            out_bag.write(topic, msg, t)
print 'Done. Re-compression needed.'
