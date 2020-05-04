idf_component_register(SRCS 
"src/error.c"
"src/fifo.c"   
"src/ieee488.c"
"src/lexer_private.h"
"src/parser.c"
"src/scpi.g"
"src/utils.c"
"src/expression.c"
"src/fifo_private.h"
"src/lexer.c"
"src/minimal.c"
"src/parser_private.h"
"src/units.c"
"src/utils_private.h"
 INCLUDE_DIRS "inc")

#set(COMPONENT_SRCS 
#src/error.c       
#src/fifo.c          
#src/ieee488.c  
#src/lexer_private.h  
#src/parser.c          
#src/scpi.g   
#src/utils.c
#src/expression.c  
#src/fifo_private.h  
#src/lexer.c    
#src/minimal.c        
#src/parser_private.h  
#src/units.c  
#src/utils_private.h
#)
#set(COMPONENT_ADD_INCLUDEDIRS "inc")
#register_component()
