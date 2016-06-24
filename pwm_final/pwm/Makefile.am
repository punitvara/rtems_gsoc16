rtems_tests_PROGRAMS = pwm
pwm_SOURCES = init.c

dist_rtems_tests_DATA = pwm.scn
dist_rtems_tests_DATA += pwm.doc

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am


LINK_OBJS = $(pwm_OBJECTS)
LINK_LIBS = $(pwm_LDLIBS)

hello$(EXEEXT): $(pwm_OBJECTS) $(pwm_DEPENDENCIES)
	@rm -f pwm$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
