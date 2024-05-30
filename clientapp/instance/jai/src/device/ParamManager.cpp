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
  auto type = getParamType(params, paramName);
  if (type != PvGenType::PvGenTypeUndefined) {
    return true;
  }
  return false;
}

PvGenType ParamManager::getParamType(PvGenParameterArray* params,
                                     const char* paramName) {
  auto param = params->Get(paramName);
  if (param) {
    PvGenType type;
    param->GetType(type);
    Debug << "Parameter " << paramName << " found with type " << type;
    return type;
  }
  ErrorLog << "Parameter " << paramName << " not found";
  return PvGenType::PvGenTypeUndefined;
}

void ParamManager::printPvResult(PvResult result) {
  if (!result.IsOK()) {
    ErrorLog << "PvResult: " << result.GetCodeString().GetAscii();
  }
}

std::string ParamManager::getParameterAsString(PvGenParameterArray* params,
                                               const char* paramName) {
  auto type = getParamType(params, paramName);
  PvString s_value;
  switch (type) {
    case PvGenType::PvGenTypeFloat:
      double d_value;
      params->GetFloatValue(paramName, d_value);
      return std::to_string(d_value);
    case PvGenType::PvGenTypeInteger:
      int64_t i_value;
      params->GetIntegerValue(paramName, i_value);
      return std::to_string(i_value);
    case PvGenType::PvGenTypeBoolean:
      bool b_value;
      params->GetBooleanValue(paramName, b_value);
      return std::to_string(b_value);
    case PvGenType::PvGenTypeString:

      params->GetStringValue(paramName, s_value);
      return s_value.GetAscii();
    case PvGenType::PvGenTypeEnum:
      int64_t value;
      params->GetEnumValue(paramName, value);
      return std::to_string(value);
    default:
      return "";
  }
}