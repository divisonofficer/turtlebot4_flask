import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { sensorsStore, Sensor, SensorGroup } from "../../stores/SensorsStore";
import {
  Card,
  Flex,
  HStack,
  IconButton,
  Input,
  Switch,
  VStack,
} from "@chakra-ui/react";
import { CameraPlus, Pause, Play } from "@phosphor-icons/react";
import { Body2, Body3, H4 } from "../../design/text/textsystem";
import { useEffect, useRef, useState } from "react";
import { lucidSocket } from "../../connect/socket/subscribe";
import { Btn } from "../../design/button/button";
import { lucidStore } from "../../stores/LucidStore";

const SensorCameraControl = observer((props: { sensor: Sensor.Camera }) => {
  return (
    <HStack>
      {props.sensor.state.stream_on || (
        <IconButton
          icon={<Play />}
          aria-label=""
          onClick={() => sensorsStore.stream_start(props.sensor)}
        />
      )}
      {props.sensor.state.stream_on && (
        <IconButton
          icon={<Pause />}
          aria-label=""
          onClick={() => sensorsStore.stream_stop(props.sensor)}
        />
      )}
    </HStack>
  );
});

const SensorPreview = observer((props: { sensor: Sensor.Sensor }) => {
  const [preview, setPreview] = useState<(string | undefined)[]>(
    new Array(props.sensor.const.preview_keys.length).fill(undefined)
  );

  const canvasRefs = useRef<(HTMLCanvasElement | null)[]>([]);

  useEffect(() => {
    const subscriptions = props.sensor.const.preview_keys.map((key, idx) => {
      lucidSocket.subscribe(
        `/thumb/${props.sensor.name}/${key}`,
        (encoded: string) => {
          const decodedImage = `data:image/jpeg;base64,${encoded}`;
          setPreview((prev) => {
            const newPrev = [...prev];
            newPrev[idx] = decodedImage;
            return newPrev;
          });
        }
      );
      return key;
    });

    return () => {
      subscriptions.forEach((sub) => lucidSocket.unsubscribe(sub));
    };
  }, [props.sensor]);

  useEffect(() => {
    preview.forEach((src, idx) => {
      const canvas = canvasRefs.current[idx];
      if (canvas && src) {
        const ctx = canvas.getContext("2d");
        if (ctx) {
          const image = new Image();
          image.src = src;
          image.onload = () => {
            canvas.width = image.width;
            canvas.height = image.height;
            ctx.drawImage(image, 0, 0);
          };
        }
      }
    });
  }, [preview]);

  return (
    <Card style={{ width: "16rem", margin: "1rem", position: "relative" }}>
      {props.sensor.const.preview_keys.map((_, idx) => (
        <canvas
          key={idx}
          ref={(el) => (canvasRefs.current[idx] = el)}
          style={{
            width: "100%",
            height: "100%",
            display: "block",
            marginBottom: "0.5rem",
          }}
        />
      ))}
      <IconButton
        icon={props.sensor.state.preview_on ? <Pause /> : <Play />}
        aria-label="Pause"
        style={{ position: "absolute", bottom: "0.5rem", right: "0.5rem" }}
        onClick={() => {
          props.sensor.state.preview_on
            ? sensorsStore.fetchPostPreviewOff(props.sensor)
            : sensorsStore.fetchPostPreviewOn(props.sensor);
        }}
      />
    </Card>
  );
});

const SensorStateView = observer(({ state }: { state: Sensor.State }) => {
  return (
    <VStack>
      <HStack>
        <Card
          style={{
            width: "8rem",
            backgroundColor: "#f0f0f0",
            display: "flex",
            flexDirection: "column",
            justifyContent: "space-between",
            height: "4rem",
            padding: "0.5rem",
          }}
        >
          <H4 style={{ fontSize: "0.75rem", textAlign: "left" }}>Last</H4>
          {"timestamp_last" in state && state.timestamp_last && (
            <H4 style={{ textAlign: "right" }}>
              {new Date(state.timestamp_last * 1000)
                .toISOString()
                .substr(14, 9)}
            </H4>
          )}
        </Card>
        <Card
          style={{
            width: "8rem",
            backgroundColor: "#e0e0e0",
            display: "flex",
            flexDirection: "column",
            justifyContent: "space-between",
            height: "4rem",
            padding: "0.5rem",
          }}
        >
          <H4 style={{ fontSize: "0.75rem", textAlign: "left" }}>FPS</H4>
          <H4 style={{ textAlign: "right" }}>{state.fps}</H4>
        </Card>
      </HStack>
    </VStack>
  );
});

const SensorConfigView = observer((props: { sensor: Sensor.Sensor }) => {
  const params = props.sensor.config || {};
  const [pendingValues, setPendingValues] = useState<Record<string, any>>({});

  // Initialize pendingValues with current values
  useEffect(() => {
    const initialValues: Record<string, any> = {};
    Object.entries(params).forEach(([name, param]) => {
      initialValues[name] = param.value;
    });
    setPendingValues(initialValues);
  }, [params]);

  const updateConfig = (name: string, value: any) => {
    // Dummy function for now - will be implemented later
    // console.log(`Updating ${name} to ${value}`);
    // // Update the pending value
    // setPendingValues((prev) => ({ ...prev, [name]: value }));
    sensorsStore.updateSensorConfig(props.sensor, name, value);
  };

  const ParamInt = (props: { name: string; param: Sensor.Param.IntParam }) => {
    const { name, param } = props;
    const [localValue, setLocalValue] = useState<number>(param.value);

    return (
      <Input
        fontSize="sm"
        p={0.5}
        height="1.6rem"
        type="number"
        min={param.min}
        max={param.max}
        width="8rem"
        value={localValue}
        onChange={(e) => setLocalValue(Number(e.target.value))}
        onKeyDown={(e) => {
          if (e.key === "Enter") {
            updateConfig(name, localValue);
          }
        }}
        onBlur={() => updateConfig(name, localValue)}
      />
    );
  };

  const ParamFloat = (props: {
    name: string;
    param: Sensor.Param.FloatParam;
  }) => {
    const { name, param } = props;
    const [localValue, setLocalValue] = useState<number>(param.value);

    return (
      <Input
        fontSize="sm"
        p={0.5}
        height="1.6rem"
        type="number"
        min={param.min}
        max={param.max}
        width="8rem"
        value={localValue}
        onChange={(e) => setLocalValue(Number(e.target.value))}
        onKeyDown={(e) => {
          if (e.key === "Enter") {
            updateConfig(name, localValue);
          }
        }}
        onBlur={() => updateConfig(name, localValue)}
      />
    );
  };

  const ParamBool = (props: {
    name: string;
    param: Sensor.Param.BoolParam;
  }) => {
    const { name, param } = props;
    return (
      <Switch
        size="sm"
        isChecked={param.value}
        onChange={(e) => updateConfig(name, e.target.checked)}
      />
    );
  };

  const ParamEnum = (props: {
    name: string;
    param: Sensor.Param.EnumParam;
  }) => {
    const { name, param } = props;
    return (
      <select
        style={{
          width: "4rem",
          fontSize: "0.75rem",
        }}
        value={param.value}
        onChange={(e) => updateConfig(name, e.target.value)}
      >
        {param.value_list.map((v, i) => {
          return (
            <option
              key={i}
              style={{
                fontSize: "0.75rem",
              }}
            >
              {v}
            </option>
          );
        })}
      </select>
    );
  };

  const ParamControl = (name: string, param: Sensor.Param.Param) => {
    if (param.type === "int") {
      return <ParamInt name={name} param={param as Sensor.Param.IntParam} />;
    }
    if (param.type === "float") {
      return (
        <ParamFloat name={name} param={param as Sensor.Param.FloatParam} />
      );
    }
    if (param.type === "bool") {
      return <ParamBool name={name} param={param as Sensor.Param.BoolParam} />;
    }
    if (param.type === "enum") {
      return <ParamEnum name={name} param={param as Sensor.Param.EnumParam} />;
    }
    return null;
  };

  return (
    <VStack
      style={{
        width: "100%",
      }}
    >
      {Object.entries(params)
        .filter(([name, param]) => param.hidden !== true)
        .map(([name, param], i) => {
          return (
            <HStack
              key={i}
              width="100%"
              justifyContent="space-between"
              height="2rem"
            >
              <Body3>{name}</Body3>
              {ParamControl(name, param)}
            </HStack>
          );
        })}
    </VStack>
  );
});

const SensorView = observer((props: { sensor: Sensor.Sensor }) => {
  return (
    <Card
      style={{
        width: "16rem",
        margin: "1rem",
      }}
    >
      <VStack>
        <H4
          style={{
            textAlign: "left",
            width: "100%",
            overflow: "hidden",
          }}
        >
          {props.sensor.name}
        </H4>

        {props.sensor.type === "camera" && (
          <SensorCameraControl sensor={props.sensor as Sensor.Camera} />
        )}
        {props.sensor.const.preview_enabled && (
          <SensorPreview sensor={props.sensor} />
        )}
        {props.sensor.type === "camera" && (
          <SensorStateView state={(props.sensor as Sensor.Camera).state} />
        )}
        <SensorConfigView sensor={props.sensor} />
      </VStack>
    </Card>
  );
});

const SensorGroupView = observer((props: { groups: SensorGroup[] }) => {
  useEffect(() => {
    sensorsStore.fetchGetGroupsList();
  }, []);
  return (
    <Flex>
      {props.groups.map((group, i) => {
        return (
          <Card
            key={i}
            style={{
              width: "16rem",
              margin: "1rem",
            }}
          >
            <VStack>
              <H4
                style={{
                  textAlign: "left",
                  width: "100%",
                  overflow: "hidden",
                }}
              >
                {group.name}
              </H4>
              {group.state.trigger_loop_on || (
                <IconButton
                  icon={<Play />}
                  aria-label=""
                  onClick={() => sensorsStore.fetchGroupTriggerLoopOn(group)}
                />
              )}
              {group.state.trigger_loop_on && (
                <IconButton
                  icon={<Pause />}
                  aria-label=""
                  onClick={() => sensorsStore.fetchGroupTrigger(group)}
                />
              )}
              <IconButton
                icon={<CameraPlus />}
                aria-label=""
                onClick={() => sensorsStore.fetchGroupTrigger(group)}
              />
            </VStack>
          </Card>
        );
      })}
    </Flex>
  );
});

export const SensorsPage = observer(() => {
  useEffect(() => {
    sensorsStore.fetchGetSensorsList();
  }, []);

  return (
    <PageRoot title="Sensors">
      <Flex>
        {sensorsStore.sensors.map((sensor, i) => {
          return <SensorView key={i} sensor={sensor} />;
        })}
      </Flex>
      <SensorGroupView groups={sensorsStore.sensorGroups} />

      {lucidStore.storageEnabled ? (
        <Btn onClick={lucidStore.fetchDisableStorage}>Disable Storage</Btn>
      ) : (
        <Btn onClick={lucidStore.fetchEnableStorage}>Enable Storage</Btn>
      )}
      {lucidStore.lucidStatus.hasOwnProperty("single_storage_mode") && (
        <Switch
          isChecked={lucidStore.lucidStatus["single_storage_mode"]}
          onChange={(e) => {
            lucidStore.fetchUpdateStatusAttributes({
              single_storage_mode: e.target.checked,
            });
          }}
        />
      )}
    </PageRoot>
  );
});
