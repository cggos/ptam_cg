# DO NOT DELETE THIS LINE -- make depend depends on it.


# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++
COMPILEFLAGS = -I -D_LINUX -D_REENTRANT -Wall  -O3 -march=nocona -msse3 -I/usr/include/eigen3 -I/usr/include/suitesparse -std=c++11 -g
LINKFLAGS = -L -lX11 -ltiff  -lcxsparse -lg2o_solver_csparse -lg2o_types_sba -lcholmod -lg2o_solver_eigen -lg2o_stuff -lg2o_core -lg2o_csparse_extension -lg2o_solver_cholmod -ljpeg -lpng -lglut -lGLU -lGL -ldc1394 -lXext -pthread -lGVars3 -lcvd -llapack -lcblas -lrefblas -ltmglib -lgfortran

# Edit this line to change video source
VIDEOSOURCE = VideoSource_Linux_V4L.o

OBJECTS=	main.o\
		GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		System.o \
		ATANCamera.o\
		KeyFrame.o\
		MapPoint.o\
		Map.o\
		SmallBlurryImage.o\
		ShiTomasi.o \
		HomographyInit.o \
		MapMaker.o \
		Bundle.o \
		PatchFinder.o\
		Relocaliser.o\
		MiniPatch.o\
		MapViewer.o\
		ARDriver.o\
		EyeGame.o\
		Tracker.o

CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o

All: PTAM CameraCalibrator

PTAM: $(OBJECTS)
	$(CC) -o PTAM $(OBJECTS) $(LINKFLAGS)

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -o CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS)


%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS)

clean:
	rm *.o PTAM CameraCalibrator


depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.h


-include dependencies
