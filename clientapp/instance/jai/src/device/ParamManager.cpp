#include <Logger.h>
#include <ParamManager.h>

PvResult ParamManager::setParam(PvGenParameterArray* params,
                                const char* paramName, const int64_t value) {
  Debug << "Setting param " << paramName << " to " << value;
  PvResult lResult;
  if (!isParamExists(params, paramName)) {
    return PvResult::Code::NOT_FOUND;
  }
  lResult = params->SetIntegerValue(paramName, value);
  printPvResult(lResult);
  return lResult;
}

PvResult ParamManager::setParam(PvGenParameterArray* params,
                                const char* paramName, const char* value) {
  Debug << "Setting param " << paramName << " to " << value;
  PvResult lResult;
  if (!isParamExists(params, paramName)) {
    return PvResult::Code::NOT_FOUND;
  }
  lResult = params->SetStringValue(paramName, value);
  printPvResult(lResult);
  return lResult;
}

PvResult ParamManager::setParam(PvGenParameterArray* params,
                                const char* paramName, const bool value) {
  Debug << "Setting param " << paramName << " to " << value;
  PvResult lResult;
  if (!isParamExists(params, paramName)) {
    return PvResult::Code::NOT_FOUND;
  }
  lResult = params->SetBooleanValue(paramName, value);
  printPvResult(lResult);
  return lResult;
}

PvResult ParamManager::setParam(PvGenParameterArray* params,
                                const char* paramName, const float value) {
  Debug << "Setting param " << paramName << " to " << value;
  PvResult lResult;
  if (!isParamExists(params, paramName)) {
    return PvResult::Code::NOT_FOUND;
  }
  lResult = params->SetFloatValue(paramName, value);
  printPvResult(lResult);
  return lResult;
}

PvResult ParamManager::setParamEnum(PvGenParameterArray* params,
                                    const char* paramName, const int value) {
  Debug << "Setting param " << paramName << " to " << value;
  PvResult lResult;
  if (!isParamExists(params, paramName)) {
    return PvResult::Code::NOT_FOUND;
  }
  lResult = params->SetEnumValue(paramName, value);
  printPvResult(lResult);
  return lResult;
}

PvResult ParamManager::setParam(PvGenParameterArray* params,
                                const char* paramName, const u_int32_t value) {
  return setParam(params, paramName, static_cast<int64_t>(value));
}

PvResult ParamManager::setParam(PvGenParameterArray* params,
                                const char* paramName, const int value) {
  return setParam(params, paramName, static_cast<int64_t>(value));
}

bool ParamManager::isParamExists(PvGenParameterArray* params,
                                 const char* paramName) {
  auto param = params->Get(paramName);
  if (param) {
    PvGenType type;
    param->GetType(type);
    Debug << "Parameter " << paramName << " found with type " << type;
    return true;
  }
  ErrorLog << "Parameter " << paramName << " not found";
  return false;
}

void ParamManager::printPvResult(PvResult result) {
  if (!result.IsOK()) {
    ErrorLog << "PvResult: " << result.GetCodeString().GetAscii();
  }
}