/// debug
#ifndef DBG_H_
#define DBG_H_
#include <stdlib.h>
#include <iostream>
#if 1
	//#define dbg(T) 		std::cout << T << std::endl;  
	#define dbg(T) 		printf(T); 
	 
	/// https://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html	
	//#define dbg2(T,v)	std::cout << T << v << std::endl;  
	#define dbg2(T,...)	printf(T,__VA_ARGS__);  
#else
	#define dbg(T) 
	#define dbg2(T,...) 
#endif

#endif
