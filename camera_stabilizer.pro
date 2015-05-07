TEMPLATE=subdirs

SUBDIRS+=\
    stabilizer test_stabilization  \
    alt_stabilizer \
    mser_tools

stabilizer.depends=mser_tools
test_stabilization.depends=stabilizer
