# 定义编译器和编译选项
CXX = clang++
CXXFLAGS = -fPIC -DBSD=1 -DSOCKLEN_T=socklen_t -DHAVE_SOCKADDR_LEN=1 -DTIME_BASE=int -DNEED_XLOCALE_H=1 -I../include -I/opt/homebrew/opt/openssl@3/include -O2
LDFLAGS = -shared -L/opt/homebrew/opt/openssl@3/lib -lssl -lcrypto -lpthread

# 输出动态库的路径
OUTPUT_DIR = /Users/sunday/Desktop/Code/rtspClient/lib

# 定义子目录和生成的动态库名称
SUBDIRS = BasicUsageEnvironment groupsock liveMedia UsageEnvironment
LIBS = $(OUTPUT_DIR)/libliveMedia.dylib $(OUTPUT_DIR)/libgroupsock.dylib $(OUTPUT_DIR)/libBasicUsageEnvironment.dylib $(OUTPUT_DIR)/libUsageEnvironment.dylib

# 默认规则
all: $(LIBS)

# 为每个子目录生成动态库
$(OUTPUT_DIR)/libliveMedia.dylib:
	@echo "Building libliveMedia.dylib"
	$(MAKE) -C liveMedia CXX=$(CXX) CXXFLAGS="$(CXXFLAGS)" LDFLAGS="$(LDFLAGS) -o $(OUTPUT_DIR)/libliveMedia.dylib"

$(OUTPUT_DIR)/libgroupsock.dylib:
	@echo "Building libgroupsock.dylib"
	$(MAKE) -C groupsock CXX=$(CXX) CXXFLAGS="$(CXXFLAGS)" LDFLAGS="$(LDFLAGS) -o $(OUTPUT_DIR)/libgroupsock.dylib"

$(OUTPUT_DIR)/libBasicUsageEnvironment.dylib:
	@echo "Building libBasicUsageEnvironment.dylib"
	$(MAKE) -C BasicUsageEnvironment CXX=$(CXX) CXXFLAGS="$(CXXFLAGS)" LDFLAGS="$(LDFLAGS) -o $(OUTPUT_DIR)/libBasicUsageEnvironment.dylib"

$(OUTPUT_DIR)/libUsageEnvironment.dylib:
	@echo "Building libUsageEnvironment.dylib"
	$(MAKE) -C UsageEnvironment CXX=$(CXX) CXXFLAGS="$(CXXFLAGS)" LDFLAGS="$(LDFLAGS) -o $(OUTPUT_DIR)/libUsageEnvironment.dylib"

# 清理生成的文件
clean:
	for dir in $(SUBDIRS); do \
		$(MAKE) -C $$dir clean; \
	done
	rm -f $(LIBS)