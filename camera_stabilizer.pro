TEMPLATE=subdirs

SUBDIRS+=\
    stabilizer test_stabilization  \
    mser_tools

stabilizer.depends=mser_tools
test_stabilization.depends=stabilizer
