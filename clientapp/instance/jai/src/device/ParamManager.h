#include <PvGenParameterArray.h>
#include <PvResult.h>

class ParamManager {
 public:
  static PvResult setParam(PvGenParameterArray* params, const char* paramName,
                           const int64_t value);
  static PvResult setParam(PvGenParameterArray* params, const char* paramName,
                           const char* value);
  static PvResult setParam(PvGenParameterArray* params, const char* paramName,
                           const bool value);
  static PvResult setParam(PvGenParameterArray* params, const char* paramName,
                           const u_int32_t value);
  static PvResult setParam(PvGenParameterArray* params, const char* paramName,
                           const int value);

  static PvResult setParam(PvGenParameterArray* params, const char* paramName,
                           const float value);

  static PvResult setParamEnum(PvGenParameterArray* params,
                               const char* paramName, const int value);

  static bool isParamExists(PvGenParameterArray* params, const char* paramName);
  static void printPvResult(PvResult result);
};