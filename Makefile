all:
	cd thirdparty && make
	make -f Makefile_package

.DEFAULT:
	cd thirdparty && make $@
	make -f Makefile_package $@


