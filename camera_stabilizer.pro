TEMPLATE=subdirs

SUBDIRS+=\
    test_stabilization
SUBDIRS+=stabilizer

test_stabilization.depends=stabilizer
