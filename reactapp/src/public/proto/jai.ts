// Code generated by protoc-gen-ts_proto. DO NOT EDIT.
// versions:
//   protoc-gen-ts_proto  v1.174.0
//   protoc               v3.12.4
// source: jai.proto

/* eslint-disable */
import * as _m0 from "protobufjs/minimal";

export const protobufPackage = "";

export interface ParameterInfo {
  name: string;
  type: string;
  min: number;
  max: number;
  source: ParameterInfo_Source;
  enumDefs: ParameterEnum[];
}

export enum ParameterInfo_Source {
  DEVICE = 0,
  SOURCE = 1,
  STREAM = 2,
  STRMAM_GEV = 3,
  UNRECOGNIZED = -1,
}

export function parameterInfo_SourceFromJSON(object: any): ParameterInfo_Source {
  switch (object) {
    case 0:
    case "DEVICE":
      return ParameterInfo_Source.DEVICE;
    case 1:
    case "SOURCE":
      return ParameterInfo_Source.SOURCE;
    case 2:
    case "STREAM":
      return ParameterInfo_Source.STREAM;
    case 3:
    case "STRMAM_GEV":
      return ParameterInfo_Source.STRMAM_GEV;
    case -1:
    case "UNRECOGNIZED":
    default:
      return ParameterInfo_Source.UNRECOGNIZED;
  }
}

export function parameterInfo_SourceToJSON(object: ParameterInfo_Source): string {
  switch (object) {
    case ParameterInfo_Source.DEVICE:
      return "DEVICE";
    case ParameterInfo_Source.SOURCE:
      return "SOURCE";
    case ParameterInfo_Source.STREAM:
      return "STREAM";
    case ParameterInfo_Source.STRMAM_GEV:
      return "STRMAM_GEV";
    case ParameterInfo_Source.UNRECOGNIZED:
    default:
      return "UNRECOGNIZED";
  }
}

export interface ParameterEnum {
  index: number;
  value: string;
}

export interface ParameterValue {
  name: string;
  type: string;
  value: string;
}

export interface ParameterUpdate {
  parameters: ParameterValue[];
}

export interface SourceInfo {
  name: string;
  type: string;
  parameters: ParameterUpdate | undefined;
}

export interface DeviceInfo {
  name: string;
  sourceCount: number;
  sourceTypes: SourceInfo[];
  fps: number;
  configurable: ParameterInfo[];
}

function createBaseParameterInfo(): ParameterInfo {
  return { name: "", type: "", min: 0, max: 0, source: 0, enumDefs: [] };
}

export const ParameterInfo = {
  encode(message: ParameterInfo, writer: _m0.Writer = _m0.Writer.create()): _m0.Writer {
    if (message.name !== "") {
      writer.uint32(10).string(message.name);
    }
    if (message.type !== "") {
      writer.uint32(18).string(message.type);
    }
    if (message.min !== 0) {
      writer.uint32(29).float(message.min);
    }
    if (message.max !== 0) {
      writer.uint32(37).float(message.max);
    }
    if (message.source !== 0) {
      writer.uint32(40).int32(message.source);
    }
    for (const v of message.enumDefs) {
      ParameterEnum.encode(v!, writer.uint32(50).fork()).ldelim();
    }
    return writer;
  },

  decode(input: _m0.Reader | Uint8Array, length?: number): ParameterInfo {
    const reader = input instanceof _m0.Reader ? input : _m0.Reader.create(input);
    let end = length === undefined ? reader.len : reader.pos + length;
    const message = createBaseParameterInfo();
    while (reader.pos < end) {
      const tag = reader.uint32();
      switch (tag >>> 3) {
        case 1:
          if (tag !== 10) {
            break;
          }

          message.name = reader.string();
          continue;
        case 2:
          if (tag !== 18) {
            break;
          }

          message.type = reader.string();
          continue;
        case 3:
          if (tag !== 29) {
            break;
          }

          message.min = reader.float();
          continue;
        case 4:
          if (tag !== 37) {
            break;
          }

          message.max = reader.float();
          continue;
        case 5:
          if (tag !== 40) {
            break;
          }

          message.source = reader.int32() as any;
          continue;
        case 6:
          if (tag !== 50) {
            break;
          }

          message.enumDefs.push(ParameterEnum.decode(reader, reader.uint32()));
          continue;
      }
      if ((tag & 7) === 4 || tag === 0) {
        break;
      }
      reader.skipType(tag & 7);
    }
    return message;
  },

  fromJSON(object: any): ParameterInfo {
    return {
      name: isSet(object.name) ? globalThis.String(object.name) : "",
      type: isSet(object.type) ? globalThis.String(object.type) : "",
      min: isSet(object.min) ? globalThis.Number(object.min) : 0,
      max: isSet(object.max) ? globalThis.Number(object.max) : 0,
      source: isSet(object.source) ? parameterInfo_SourceFromJSON(object.source) : 0,
      enumDefs: globalThis.Array.isArray(object?.enumDefs)
        ? object.enumDefs.map((e: any) => ParameterEnum.fromJSON(e))
        : [],
    };
  },

  toJSON(message: ParameterInfo): unknown {
    const obj: any = {};
    if (message.name !== "") {
      obj.name = message.name;
    }
    if (message.type !== "") {
      obj.type = message.type;
    }
    if (message.min !== 0) {
      obj.min = message.min;
    }
    if (message.max !== 0) {
      obj.max = message.max;
    }
    if (message.source !== 0) {
      obj.source = parameterInfo_SourceToJSON(message.source);
    }
    if (message.enumDefs?.length) {
      obj.enumDefs = message.enumDefs.map((e) => ParameterEnum.toJSON(e));
    }
    return obj;
  },

  create<I extends Exact<DeepPartial<ParameterInfo>, I>>(base?: I): ParameterInfo {
    return ParameterInfo.fromPartial(base ?? ({} as any));
  },
  fromPartial<I extends Exact<DeepPartial<ParameterInfo>, I>>(object: I): ParameterInfo {
    const message = createBaseParameterInfo();
    message.name = object.name ?? "";
    message.type = object.type ?? "";
    message.min = object.min ?? 0;
    message.max = object.max ?? 0;
    message.source = object.source ?? 0;
    message.enumDefs = object.enumDefs?.map((e) => ParameterEnum.fromPartial(e)) || [];
    return message;
  },
};

function createBaseParameterEnum(): ParameterEnum {
  return { index: 0, value: "" };
}

export const ParameterEnum = {
  encode(message: ParameterEnum, writer: _m0.Writer = _m0.Writer.create()): _m0.Writer {
    if (message.index !== 0) {
      writer.uint32(8).int32(message.index);
    }
    if (message.value !== "") {
      writer.uint32(18).string(message.value);
    }
    return writer;
  },

  decode(input: _m0.Reader | Uint8Array, length?: number): ParameterEnum {
    const reader = input instanceof _m0.Reader ? input : _m0.Reader.create(input);
    let end = length === undefined ? reader.len : reader.pos + length;
    const message = createBaseParameterEnum();
    while (reader.pos < end) {
      const tag = reader.uint32();
      switch (tag >>> 3) {
        case 1:
          if (tag !== 8) {
            break;
          }

          message.index = reader.int32();
          continue;
        case 2:
          if (tag !== 18) {
            break;
          }

          message.value = reader.string();
          continue;
      }
      if ((tag & 7) === 4 || tag === 0) {
        break;
      }
      reader.skipType(tag & 7);
    }
    return message;
  },

  fromJSON(object: any): ParameterEnum {
    return {
      index: isSet(object.index) ? globalThis.Number(object.index) : 0,
      value: isSet(object.value) ? globalThis.String(object.value) : "",
    };
  },

  toJSON(message: ParameterEnum): unknown {
    const obj: any = {};
    if (message.index !== 0) {
      obj.index = Math.round(message.index);
    }
    if (message.value !== "") {
      obj.value = message.value;
    }
    return obj;
  },

  create<I extends Exact<DeepPartial<ParameterEnum>, I>>(base?: I): ParameterEnum {
    return ParameterEnum.fromPartial(base ?? ({} as any));
  },
  fromPartial<I extends Exact<DeepPartial<ParameterEnum>, I>>(object: I): ParameterEnum {
    const message = createBaseParameterEnum();
    message.index = object.index ?? 0;
    message.value = object.value ?? "";
    return message;
  },
};

function createBaseParameterValue(): ParameterValue {
  return { name: "", type: "", value: "" };
}

export const ParameterValue = {
  encode(message: ParameterValue, writer: _m0.Writer = _m0.Writer.create()): _m0.Writer {
    if (message.name !== "") {
      writer.uint32(10).string(message.name);
    }
    if (message.type !== "") {
      writer.uint32(18).string(message.type);
    }
    if (message.value !== "") {
      writer.uint32(26).string(message.value);
    }
    return writer;
  },

  decode(input: _m0.Reader | Uint8Array, length?: number): ParameterValue {
    const reader = input instanceof _m0.Reader ? input : _m0.Reader.create(input);
    let end = length === undefined ? reader.len : reader.pos + length;
    const message = createBaseParameterValue();
    while (reader.pos < end) {
      const tag = reader.uint32();
      switch (tag >>> 3) {
        case 1:
          if (tag !== 10) {
            break;
          }

          message.name = reader.string();
          continue;
        case 2:
          if (tag !== 18) {
            break;
          }

          message.type = reader.string();
          continue;
        case 3:
          if (tag !== 26) {
            break;
          }

          message.value = reader.string();
          continue;
      }
      if ((tag & 7) === 4 || tag === 0) {
        break;
      }
      reader.skipType(tag & 7);
    }
    return message;
  },

  fromJSON(object: any): ParameterValue {
    return {
      name: isSet(object.name) ? globalThis.String(object.name) : "",
      type: isSet(object.type) ? globalThis.String(object.type) : "",
      value: isSet(object.value) ? globalThis.String(object.value) : "",
    };
  },

  toJSON(message: ParameterValue): unknown {
    const obj: any = {};
    if (message.name !== "") {
      obj.name = message.name;
    }
    if (message.type !== "") {
      obj.type = message.type;
    }
    if (message.value !== "") {
      obj.value = message.value;
    }
    return obj;
  },

  create<I extends Exact<DeepPartial<ParameterValue>, I>>(base?: I): ParameterValue {
    return ParameterValue.fromPartial(base ?? ({} as any));
  },
  fromPartial<I extends Exact<DeepPartial<ParameterValue>, I>>(object: I): ParameterValue {
    const message = createBaseParameterValue();
    message.name = object.name ?? "";
    message.type = object.type ?? "";
    message.value = object.value ?? "";
    return message;
  },
};

function createBaseParameterUpdate(): ParameterUpdate {
  return { parameters: [] };
}

export const ParameterUpdate = {
  encode(message: ParameterUpdate, writer: _m0.Writer = _m0.Writer.create()): _m0.Writer {
    for (const v of message.parameters) {
      ParameterValue.encode(v!, writer.uint32(10).fork()).ldelim();
    }
    return writer;
  },

  decode(input: _m0.Reader | Uint8Array, length?: number): ParameterUpdate {
    const reader = input instanceof _m0.Reader ? input : _m0.Reader.create(input);
    let end = length === undefined ? reader.len : reader.pos + length;
    const message = createBaseParameterUpdate();
    while (reader.pos < end) {
      const tag = reader.uint32();
      switch (tag >>> 3) {
        case 1:
          if (tag !== 10) {
            break;
          }

          message.parameters.push(ParameterValue.decode(reader, reader.uint32()));
          continue;
      }
      if ((tag & 7) === 4 || tag === 0) {
        break;
      }
      reader.skipType(tag & 7);
    }
    return message;
  },

  fromJSON(object: any): ParameterUpdate {
    return {
      parameters: globalThis.Array.isArray(object?.parameters)
        ? object.parameters.map((e: any) => ParameterValue.fromJSON(e))
        : [],
    };
  },

  toJSON(message: ParameterUpdate): unknown {
    const obj: any = {};
    if (message.parameters?.length) {
      obj.parameters = message.parameters.map((e) => ParameterValue.toJSON(e));
    }
    return obj;
  },

  create<I extends Exact<DeepPartial<ParameterUpdate>, I>>(base?: I): ParameterUpdate {
    return ParameterUpdate.fromPartial(base ?? ({} as any));
  },
  fromPartial<I extends Exact<DeepPartial<ParameterUpdate>, I>>(object: I): ParameterUpdate {
    const message = createBaseParameterUpdate();
    message.parameters = object.parameters?.map((e) => ParameterValue.fromPartial(e)) || [];
    return message;
  },
};

function createBaseSourceInfo(): SourceInfo {
  return { name: "", type: "", parameters: undefined };
}

export const SourceInfo = {
  encode(message: SourceInfo, writer: _m0.Writer = _m0.Writer.create()): _m0.Writer {
    if (message.name !== "") {
      writer.uint32(10).string(message.name);
    }
    if (message.type !== "") {
      writer.uint32(18).string(message.type);
    }
    if (message.parameters !== undefined) {
      ParameterUpdate.encode(message.parameters, writer.uint32(26).fork()).ldelim();
    }
    return writer;
  },

  decode(input: _m0.Reader | Uint8Array, length?: number): SourceInfo {
    const reader = input instanceof _m0.Reader ? input : _m0.Reader.create(input);
    let end = length === undefined ? reader.len : reader.pos + length;
    const message = createBaseSourceInfo();
    while (reader.pos < end) {
      const tag = reader.uint32();
      switch (tag >>> 3) {
        case 1:
          if (tag !== 10) {
            break;
          }

          message.name = reader.string();
          continue;
        case 2:
          if (tag !== 18) {
            break;
          }

          message.type = reader.string();
          continue;
        case 3:
          if (tag !== 26) {
            break;
          }

          message.parameters = ParameterUpdate.decode(reader, reader.uint32());
          continue;
      }
      if ((tag & 7) === 4 || tag === 0) {
        break;
      }
      reader.skipType(tag & 7);
    }
    return message;
  },

  fromJSON(object: any): SourceInfo {
    return {
      name: isSet(object.name) ? globalThis.String(object.name) : "",
      type: isSet(object.type) ? globalThis.String(object.type) : "",
      parameters: isSet(object.parameters) ? ParameterUpdate.fromJSON(object.parameters) : undefined,
    };
  },

  toJSON(message: SourceInfo): unknown {
    const obj: any = {};
    if (message.name !== "") {
      obj.name = message.name;
    }
    if (message.type !== "") {
      obj.type = message.type;
    }
    if (message.parameters !== undefined) {
      obj.parameters = ParameterUpdate.toJSON(message.parameters);
    }
    return obj;
  },

  create<I extends Exact<DeepPartial<SourceInfo>, I>>(base?: I): SourceInfo {
    return SourceInfo.fromPartial(base ?? ({} as any));
  },
  fromPartial<I extends Exact<DeepPartial<SourceInfo>, I>>(object: I): SourceInfo {
    const message = createBaseSourceInfo();
    message.name = object.name ?? "";
    message.type = object.type ?? "";
    message.parameters = (object.parameters !== undefined && object.parameters !== null)
      ? ParameterUpdate.fromPartial(object.parameters)
      : undefined;
    return message;
  },
};

function createBaseDeviceInfo(): DeviceInfo {
  return { name: "", sourceCount: 0, sourceTypes: [], fps: 0, configurable: [] };
}

export const DeviceInfo = {
  encode(message: DeviceInfo, writer: _m0.Writer = _m0.Writer.create()): _m0.Writer {
    if (message.name !== "") {
      writer.uint32(10).string(message.name);
    }
    if (message.sourceCount !== 0) {
      writer.uint32(16).int32(message.sourceCount);
    }
    for (const v of message.sourceTypes) {
      SourceInfo.encode(v!, writer.uint32(26).fork()).ldelim();
    }
    if (message.fps !== 0) {
      writer.uint32(32).int32(message.fps);
    }
    for (const v of message.configurable) {
      ParameterInfo.encode(v!, writer.uint32(42).fork()).ldelim();
    }
    return writer;
  },

  decode(input: _m0.Reader | Uint8Array, length?: number): DeviceInfo {
    const reader = input instanceof _m0.Reader ? input : _m0.Reader.create(input);
    let end = length === undefined ? reader.len : reader.pos + length;
    const message = createBaseDeviceInfo();
    while (reader.pos < end) {
      const tag = reader.uint32();
      switch (tag >>> 3) {
        case 1:
          if (tag !== 10) {
            break;
          }

          message.name = reader.string();
          continue;
        case 2:
          if (tag !== 16) {
            break;
          }

          message.sourceCount = reader.int32();
          continue;
        case 3:
          if (tag !== 26) {
            break;
          }

          message.sourceTypes.push(SourceInfo.decode(reader, reader.uint32()));
          continue;
        case 4:
          if (tag !== 32) {
            break;
          }

          message.fps = reader.int32();
          continue;
        case 5:
          if (tag !== 42) {
            break;
          }

          message.configurable.push(ParameterInfo.decode(reader, reader.uint32()));
          continue;
      }
      if ((tag & 7) === 4 || tag === 0) {
        break;
      }
      reader.skipType(tag & 7);
    }
    return message;
  },

  fromJSON(object: any): DeviceInfo {
    return {
      name: isSet(object.name) ? globalThis.String(object.name) : "",
      sourceCount: isSet(object.sourceCount) ? globalThis.Number(object.sourceCount) : 0,
      sourceTypes: globalThis.Array.isArray(object?.sourceTypes)
        ? object.sourceTypes.map((e: any) => SourceInfo.fromJSON(e))
        : [],
      fps: isSet(object.fps) ? globalThis.Number(object.fps) : 0,
      configurable: globalThis.Array.isArray(object?.configurable)
        ? object.configurable.map((e: any) => ParameterInfo.fromJSON(e))
        : [],
    };
  },

  toJSON(message: DeviceInfo): unknown {
    const obj: any = {};
    if (message.name !== "") {
      obj.name = message.name;
    }
    if (message.sourceCount !== 0) {
      obj.sourceCount = Math.round(message.sourceCount);
    }
    if (message.sourceTypes?.length) {
      obj.sourceTypes = message.sourceTypes.map((e) => SourceInfo.toJSON(e));
    }
    if (message.fps !== 0) {
      obj.fps = Math.round(message.fps);
    }
    if (message.configurable?.length) {
      obj.configurable = message.configurable.map((e) => ParameterInfo.toJSON(e));
    }
    return obj;
  },

  create<I extends Exact<DeepPartial<DeviceInfo>, I>>(base?: I): DeviceInfo {
    return DeviceInfo.fromPartial(base ?? ({} as any));
  },
  fromPartial<I extends Exact<DeepPartial<DeviceInfo>, I>>(object: I): DeviceInfo {
    const message = createBaseDeviceInfo();
    message.name = object.name ?? "";
    message.sourceCount = object.sourceCount ?? 0;
    message.sourceTypes = object.sourceTypes?.map((e) => SourceInfo.fromPartial(e)) || [];
    message.fps = object.fps ?? 0;
    message.configurable = object.configurable?.map((e) => ParameterInfo.fromPartial(e)) || [];
    return message;
  },
};

type Builtin = Date | Function | Uint8Array | string | number | boolean | undefined;

export type DeepPartial<T> = T extends Builtin ? T
  : T extends globalThis.Array<infer U> ? globalThis.Array<DeepPartial<U>>
  : T extends ReadonlyArray<infer U> ? ReadonlyArray<DeepPartial<U>>
  : T extends {} ? { [K in keyof T]?: DeepPartial<T[K]> }
  : Partial<T>;

type KeysOfUnion<T> = T extends T ? keyof T : never;
export type Exact<P, I extends P> = P extends Builtin ? P
  : P & { [K in keyof P]: Exact<P[K], I[K]> } & { [K in Exclude<keyof I, KeysOfUnion<P>>]: never };

function isSet(value: any): boolean {
  return value !== null && value !== undefined;
}
