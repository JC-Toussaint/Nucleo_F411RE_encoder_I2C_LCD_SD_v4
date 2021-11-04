#include <stdio.h>
#include <string.h>

#ifndef __DEBUG__
#define __DEBUG__

#define PRINT_MESG_DBG(...)     do{printf("\r\n [%s][%s][%d] ", DbgTraceGetFileName(__FILE__),__FUNCTION__,__LINE__);printf(__VA_ARGS__);}while(0);

const char *DbgTraceGetFileName(const char *fullpath)
{
  const char *ret = fullpath;

  if (strrchr(fullpath, '\\') != NULL)
  {
    ret = strrchr(fullpath, '\\') + 1;
  }
  else if (strrchr(fullpath, '/') != NULL)
  {
    ret = strrchr(fullpath, '/') + 1;
  }

  return ret;
}
#endif

