

INCLUDE_DIR = -Iinclude
LIBS = -lGL -lGLU -lglut -lpthread -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_calib3d
FLAGS = -g
SRCS=$(wildcard  src/*.cpp)
OBJS=$(SRCS:%.cpp=%.o)
TARGET = slam_demo


$(TARGET) : $(OBJS)
	g++ $(OBJS) $(FLAGS) $(LIBS) -o $(TARGET)

$(OBJS) : %.o : %.cpp
	g++ -c $(INCLUDE_DIR) $(LIBS) $(FLAGS) $< -o $@



clean:
	rm -f $(OBJS) $(TARGET)
	
run:
	./$(TARGET)

longxuan:
	./$(TARGET) data/videos/longxuan.mp4
	
texie:
	./$(TARGET) data/videos/texie.mp4

shanghai:
	./$(TARGET) data/videos/shanghai.mp4

gongyuan:
	./$(TARGET) data/videos/gongyuan.mp4	
	
dajiang:
	./$(TARGET) data/videos/dajiang0229.mp4
	
m1_left_8x8:
	./$(TARGET) data/images/move_left_8x8_001.png data/images/move_left_8x8_002.png

m2_left_64x64:
	./$(TARGET) data/images/move_left_64x64_001.png data/images/move_left_64x64_002.png
	
tsukuba:
	./$(TARGET) data/images/tsukuba_l.png data/images/tsukuba_r.png
	
tsukuba2:
	./$(TARGET) data/images/tsukuba_l.png data/images/tsukuba_r_rot.png

kitti:
	./$(TARGET) \
		data/images/kitti/000000.png \
		data/images/kitti/000001.png \
		data/images/kitti/000002.png \
		data/images/kitti/000003.png \
		data/images/kitti/000004.png \
		data/images/kitti/000005.png \
		data/images/kitti/000006.png \
		data/images/kitti/000007.png 		


	
debug:
	gdb ./$(TARGET)

