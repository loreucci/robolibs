add_library(sec SHARED commoncontrollers.cpp
					   commonsources.cpp
					   experimentlogger.cpp
					   sleeper.cpp
					   synchronization.cpp)
install (TARGETS sec DESTINATION lib)
install (FILES basiccontroller.h
			   commoncontrollers.h
			   commonsources.h
			   controllerfork.h
			   controller.h
			   controllersequence.h
			   experimentlogger.h
			   listener.h
			   mainthread.h
			   sleeper.h
			   source.h
			   synchronization.cpp
			   synchronization.h
			   threadslink.h
	     DESTINATION include/sec)