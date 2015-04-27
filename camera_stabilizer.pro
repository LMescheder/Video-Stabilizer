TEMPLATE=subdirs

SUBDIRS+=\
    stabilizer test_stabilization  \
    alt_stabilizer

test_stabilization.depends=stabilizer
