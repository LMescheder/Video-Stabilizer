TEMPLATE=subdirs

SUBDIRS+=\
    stabilizer libstabilizer libmsertools

libstabilizer.depends=libmsertools
stabilizer.depends=libstabilizer
