BUILD_TYPE:=RelWithDebInfo

LAUNCH_PREFIX:=valgrind
VOCAB_FILE:=../../Vocabulary/ORBvoc.txt
TIMESTAMPS_DIR:=EuRoC_TimeStamps
EUROC_DATA_DIR:=EuRoC_datasets

stereo_mh01:
	mkdir -p $@
	cd $@ && ${LAUNCH_PREFIX} \
			 ../../Stereo/${BUILD_TYPE}/stereo_euroc \
			../EuRoC.yaml \
			../${VOCAB_FILE} \
			../${EUROC_DATA_DIR}/machine_hall/MH_01_easy/ \
			../${TIMESTAMPS_DIR}/MH01.txt 2>&1 | tee stereo_mh01.txt

stereo_rerun_mh01:
	mkdir -p $@
	cd $@ && ${LAUNCH_PREFIX} \
			 ../../Stereo/${BUILD_TYPE}/stereo_euroc \
			../EuRoC_atlas.yaml \
			../${VOCAB_FILE} \
			../${EUROC_DATA_DIR}/machine_hall/MH_01_easy/ \
			../${TIMESTAMPS_DIR}/MH01.txt 2>&1 | tee stereo_mh01.txt
