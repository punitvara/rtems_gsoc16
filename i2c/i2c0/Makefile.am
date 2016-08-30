
rtems_tests_PROGRAMS = i2c0
i2c0_SOURCES = init.c

dist_rtems_tests_DATA = i2c0.scn
dist_rtems_tests_DATA += i2c0.doc

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am

LINK_OBJS = $(i2c0_OBJECTS)
LINK_LIBS = $(i2c0_LDLIBS)

i2c0$(EXEEXT): $(i2c0_OBJECTS) $(i2c0_DEPENDENCIES)
	@rm -f i2c0$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
