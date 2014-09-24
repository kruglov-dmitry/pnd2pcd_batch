PRG_NAME	=	png2pcd_batch
CPP_SRCS	:=	$(wildcard *.cpp)
OBJS		:=	${CPP_SRCS:.cpp=.o}

CXX			=	g++
WORK_DIR	=	$(HOME)

PCL_INC		=	$(WORK_DIR)/pcl-trunk/common/include \
				$(WORK_DIR)/pcl-trunk/build/include \
				$(WORK_DIR)/pcl-trunk/io/include \
				$(WORK_DIR)/pcl-trunk/common/include \
				$(WORK_DIR)/pcl-trunk/filters/include \
				$(WORK_DIR)/pcl-trunk/visualization/include \
				$(WORK_DIR)/pcl-trunk/geometry/include \
				$(WORK_DIR)/pcl-trunk/search/include \
				$(WORK_DIR)/pcl-trunk/kdtree/include \
				/usr/include/eigen3

OPENCV_INC	=	$(WORK_DIR)/opencv/modules/videoio/include/ \
				$(WORK_DIR)/opencv/modules/imgcodecs/include/ \
				$(WORK_DIR)/opencv/modules/highgui/include/ \
				$(WORK_DIR)/opencv/modules/core/include/ \
				$(WORK_DIR)/opencv/core/include/

PCL_LIB_INC		=	$(WORK_DIR)/pcl-trunk/build/lib	
OPENCV_LIB_INC	=	$(WORK_DIR)/build-opencv/lib

CPPFLAGS += $(foreach inc_dir,$(PCL_INC),-I$(inc_dir))
CPPFLAGS += $(foreach inc_dir,$(OPENCV_INC),-I$(inc_dir))

PCL_LIBS	=	pcl_io pcl_common boost_system boost_filesystem
OPENCV_LIBS	=	opencv_core opencv_highgui opencv_imgcodecs 

LDFLAGS	+=	-L$(PCL_LIB_INC) -L$(OPENCV_LIB_INC)
LDFLAGS	+=	$(foreach lib,$(PCL_LIBS),-l$(lib))
LDFLAGS	+=	$(foreach lib,$(OPENCV_LIBS),-l$(lib))

.PHONY: all clean

all: $(PRG_NAME)

$(PRG_NAME): $(OBJS)
	$(CXX) $(OBJS) -o $(PRG_NAME) $(LDFLAGS)

%.o : %.cpp
	$(CXX) $(CPPFLAGS) -c -o $@ $<

.PHONY: clean
clean:
		rm -f *.o *~ $(PRG_NAME)
