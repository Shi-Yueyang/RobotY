#ifndef ROBOTENGINE_GLOBAL_H
#define ROBOTENGINE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ROBOTENGINE_LIBRARY)
#  define ROBOTENGINE_EXPORT Q_DECL_EXPORT
#else
#  define ROBOTENGINE_EXPORT Q_DECL_IMPORT
#endif

#endif // ROBOTENGINE_GLOBAL_H
