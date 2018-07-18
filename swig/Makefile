# --------------------------------------------------------------------------
#
#	FILE NAME:		Makefile
#	AUTHOR:			UBT
#	DESCRIPTION:	SDK LIBS
#
# --------------------------------------------------------------------------
#common makefile header

SUBDIRS=python

.PHONY:
all: python_swig $(SUBDIRS) 

python_swig:
	@cp ../src/yanshee_api.c python/
	@cp ../include/yanshee_api.h python/
	@swig -python python/yanshee_api.i

$(SUBDIRS):ECHO
	make -C $@
	
ECHO: 
	@echo "###################"$(SUBDIRS)"###################"


.PHONY:clean
clean:  
	@ (                   \
		for dir in $(SUBDIRS) ; do           \
		DIR=$${dir}/;           \
		$(MAKE) -C $$DIR clean ;          \
		done                                                                    \
		)


#common makefile foot

