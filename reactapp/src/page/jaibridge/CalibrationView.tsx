import {
  Box,
  Button,
  Flex,
  HStack,
  IconButton,
  Img,
  Input,
  Menu,
  MenuButton,
  MenuItem,
  MenuList,
  Table,
  Tbody,
  Td,
  Tr,
  VStack,
  useDisclosure,
} from "@chakra-ui/react";
import { observer } from "mobx-react";
import { VideoStream } from "../../design/other/video";
import { jaiStore } from "../../stores/JaiStore";
import { Body3, H2, H3 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";
import { PageRoot } from "../../design/other/flexs";
import { useCallback, useEffect, useState } from "react";
import { Trash } from "@phosphor-icons/react";
import { ChevronDownIcon } from "@chakra-ui/icons";

const MatrixView = ({
  matrix,
  shape,
}: {
  matrix: number[];
  shape: [number, number];
}) => {
  return (
    <>
      <Table>
        <Tbody>
          {Array.from({ length: shape[0] }).map((_, i) => {
            return (
              <Tr key={i}>
                {Array.from({ length: shape[1] }).map((_, j) => {
                  return <Td key={j}>{matrix[i * shape[1] + j].toFixed(4)}</Td>;
                })}
              </Tr>
            );
          })}
        </Tbody>
      </Table>
    </>
  );
};

const CalibrationLoadBtn = observer(() => {
  useEffect(() => {
    jaiStore.fetchGetCalibrationList();
  }, []);
  return (
    <Flex
      style={{
        position: "relative",
      }}
    >
      <Menu placement="bottom-end">
        <MenuButton as={Button} rightIcon={<ChevronDownIcon />}>
          LoadProfile
        </MenuButton>
        <MenuList>
          {jaiStore.calibrationList.map((meta, index) => {
            return (
              <MenuItem
                key={index}
                fontSize="small"
                onClick={() => {
                  jaiStore.fetchCalibrationLoad(meta.id);
                }}
              >
                {meta.id} : {meta.month}/{meta.day} {meta.hour}
              </MenuItem>
            );
          })}
        </MenuList>
      </Menu>
    </Flex>
  );
});

const CalibrationChessboardShapeView = observer(() => {
  const [rows, setRows] = useState(jaiStore.chessboardShape.rows);
  const [cols, setCols] = useState(jaiStore.chessboardShape.cols);
  const [length, setLength] = useState(jaiStore.chessboardShape.length);

  return (
    <HStack>
      <Body3>Rows</Body3>
      <Input
        defaultValue={rows}
        onChange={(e) => setRows(parseInt(e.target.value))}
        style={{
          width: "3rem",
        }}
      />
      <Body3>Cols</Body3>
      <Input
        defaultValue={cols}
        onChange={(e) => setCols(parseInt(e.target.value))}
        style={{
          width: "3rem",
        }}
      />
      <Body3>Length</Body3>
      <Input
        defaultValue={length}
        onChange={(e) => setLength(parseInt(e.target.value))}
        style={{
          width: "3rem",
        }}
      />
      <Btn
        onClick={() =>
          jaiStore.fetchUpdateCalibrationChessboard({
            rows,
            cols,
            length,
          })
        }
        size="sm"
      >
        Set
      </Btn>
    </HStack>
  );
});

const CalibrationResultView = observer(() => {
  const ImagePairListView = useCallback((props: { imageCount: number }) => {
    return (
      <VStack
        style={{
          width: "24rem",
          maxHeight: "48rem",
          overflowY: "auto",
        }}
      >
        {Array.from({ length: props.imageCount }).map((_, i) => {
          return (
            <HStack
              style={{
                width: "100%",
              }}
              onClick={() => setVIdx(i)}
              key={i}
            >
              <Img
                src={`/jai/calibrate/chessboard/${i}/0`}
                style={{ width: "45%" }}
              />
              <Img
                src={`/jai/calibrate/chessboard/${i}/1`}
                style={{ width: "45%" }}
              />
            </HStack>
          );
        })}
      </VStack>
    );
  }, []);

  const [vIdx, setVIdx] = useState(0);

  const ReprojectionErrorGraph = (props: {
    vIdx: number;
    errorLeft: number[];
    errorRight: number[];
  }) => {
    const errorMax = Math.max(...props.errorLeft, ...props.errorRight);
    const errorMin = 0;
    const length = props.errorLeft.length;

    const graphVGap = errorMax > 1 ? 1 : errorMax > 0.1 ? 0.1 : 0.01;
    const graphVGapCount = Math.floor(errorMax / graphVGap) + 1;

    return (
      <Flex
        style={{
          height: "24rem",
          flexGrow: 1,
          position: "relative",
        }}
      >
        <HStack
          style={{
            height: "100%",
            width: "100%",
            alignItems: "flex-end",
            overflowX: "auto",
          }}
        >
          <VStack gap={0} height="100%">
            {graphVGapCount > 0 &&
              Array.from(Array(graphVGapCount)).map((_, i) => (
                <Flex
                  style={{
                    height: `${100 / (errorMax / graphVGap)}%`,
                    flexDirection: "column",
                    justifyContent: "flex-end",
                  }}
                >
                  <Body3>
                    {(graphVGap * (graphVGapCount - 1 - i)).toFixed(2)}
                  </Body3>
                  <div
                    style={{
                      width: "100%",
                      height: "1px",
                      background: "black",
                      opacity: 0.5,
                    }}
                  />
                </Flex>
              ))}
            <Body3>Error</Body3>
          </VStack>

          {Array.from({ length }).map((_, i) => (
            <VStack
              style={{
                height: "100%",
                gap: 0,
              }}
              onClick={() => setVIdx(i)}
            >
              <HStack
                style={{
                  flexGrow: 1,
                  maxWidth: "1rem",
                  alignItems: "flex-end",
                }}
              >
                <div
                  style={{
                    width: "0.5rem",
                    height: `${
                      ((props.errorLeft[i] - errorMin) /
                        (errorMax - errorMin)) *
                      100
                    }%`,
                    background: "red",
                    opacity: vIdx === i ? 1 : 0.5,
                  }}
                />
                <div
                  style={{
                    width: "0.5rem",
                    height: `${
                      ((props.errorRight[i] - errorMin) /
                        (errorMax - errorMin)) *
                      100
                    }%`,
                    background: "blue",
                    opacity: vIdx === i ? 1 : 0.5,
                  }}
                />
              </HStack>
              <Body3>{i}</Body3>
            </VStack>
          ))}
        </HStack>
      </Flex>
    );
  };

  return (
    <VStack
      style={{
        padding: "2rem",
      }}
    >
      <HStack style={{ width: "100%" }}>
        <ImagePairListView
          imageCount={jaiStore.stereoMatrix!.left!.reprojectError.length}
        />
        <VStack>
          <HStack>
            <H2>Image {vIdx}</H2>

            <IconButton
              icon={<Trash />}
              onClick={() =>
                window.confirm("Are you sure you want to delete this image?") &&
                jaiStore.deleteCalibrationImage(vIdx)
              }
              aria-label=""
            />
          </HStack>
          <HStack>
            <Img
              src={`/jai/calibrate/chessboard/${vIdx}/0?timestamp=${Date.now()}`}
              style={{ width: "50%" }}
            />
            <Img
              src={`/jai/calibrate/chessboard/${vIdx}/1?timestamp=${Date.now()}`}
              style={{ width: "50%" }}
            />
          </HStack>
          <HStack>
            <Body3>Reprojection Error</Body3>
            <Body3>
              {(jaiStore.stereoMatrix!.left?.reprojectError?.length ??
                0 > vIdx) &&
                jaiStore.stereoMatrix!.left!.reprojectError[vIdx].toFixed(4)}
            </Body3>
            <Body3>
              {(jaiStore.stereoMatrix!.right?.reprojectError?.length ??
                0 > vIdx) &&
                jaiStore.stereoMatrix!.right!.reprojectError[vIdx].toFixed(4)}
            </Body3>
          </HStack>
          <ReprojectionErrorGraph
            vIdx={vIdx}
            errorLeft={jaiStore.stereoMatrix!.left!.reprojectError}
            errorRight={jaiStore.stereoMatrix!.right!.reprojectError}
          />
        </VStack>
      </HStack>
    </VStack>
  );
});

const CalibrationMatrixView = observer(() => {
  return (
    <Flex wrap="wrap">
      <Btn onClick={() => jaiStore.downloadCalibrationJson()}>
        Download Parameter
      </Btn>

      {jaiStore.stereoMatrix && (
        <>
          <VStack>
            <H3>Left_MTX</H3>
            <MatrixView
              matrix={jaiStore.stereoMatrix.left!.mtx}
              shape={[3, 3]}
            />
          </VStack>
          <VStack>
            <H3>Left_Dist</H3>
            <MatrixView
              matrix={jaiStore.stereoMatrix.left!.dist}
              shape={[1, 5]}
            />
          </VStack>{" "}
          <VStack>
            <H3>Right_MTX</H3>
            <MatrixView
              matrix={jaiStore.stereoMatrix.right!.mtx}
              shape={[3, 3]}
            />
          </VStack>{" "}
          <VStack>
            <H3>Right_Dist</H3>
            <MatrixView
              matrix={jaiStore.stereoMatrix.right!.dist}
              shape={[1, 5]}
            />
          </VStack>{" "}
          <VStack>
            <H3>Rotation</H3>
            <MatrixView matrix={jaiStore.stereoMatrix.R} shape={[3, 3]} />
          </VStack>{" "}
          <VStack>
            <H3>Translation</H3>
            <MatrixView matrix={jaiStore.stereoMatrix.T} shape={[1, 3]} />
          </VStack>{" "}
          <VStack>
            <H3>Essential</H3>
            <MatrixView matrix={jaiStore.stereoMatrix.E} shape={[3, 3]} />
          </VStack>{" "}
          <VStack>
            <H3>Fundamental</H3>
            <MatrixView matrix={jaiStore.stereoMatrix.F} shape={[3, 3]} />
          </VStack>
        </>
      )}
    </Flex>
  );
});

export const LidarTransformView = observer(() => {
  const [transform, setTransform] = useState(jaiStore.lidar_transform || []);

  useEffect(() => {
    setTransform(jaiStore.lidar_transform);
  }, [jaiStore.lidar_transform]);

  useEffect(() => {
    jaiStore.fetchLidarTransform();
  }, []);

  return (
    <HStack>
      <Box w="100%" p={4} color="black">
        <Table variant="simple">
          <Tbody>
            {[...Array(3)].map((_, rowIndex) => (
              <Tr key={rowIndex}>
                {[...Array(4)].map((_, colIndex) => (
                  <Td
                    key={colIndex}
                    style={{
                      padding: "0rem",
                    }}
                  >
                    <Input
                      fontSize="small"
                      style={{
                        width: "6rem",
                        margin: "0rem",
                        padding: "0.2rem",
                      }}
                      defaultValue={transform[rowIndex][colIndex]}
                      onChange={(e) => {
                        const newTransform = transform;
                        newTransform[rowIndex][colIndex] = parseFloat(
                          e.target.value
                        );
                        setTransform(newTransform);
                      }}
                    />
                  </Td>
                ))}
              </Tr>
            ))}
          </Tbody>
        </Table>
      </Box>
      <Btn
        onClick={() => {
          jaiStore.updateLidarTransform(transform);
        }}
        size="sm"
      >
        Update
      </Btn>
    </HStack>
  );
});

export const CalibrateView = observer(() => {
  return (
    <VStack>
      <Btn size="sm" onClick={() => jaiStore.calibrationCapture()}>
        Capture
      </Btn>
      <HStack>
        <Btn size="sm" onClick={() => jaiStore.fetchCalibrationSave()}>
          Save
        </Btn>
        <CalibrationLoadBtn />
        <CalibrationChessboardShapeView />
        {/*<LidarTransformView /> */}
      </HStack>

      <HStack
        style={{
          overflowX: "auto",
        }}
      >
        <VideoStream
          url={`/jai/calibrate/videostream`}
          style={{
            width: "48rem",
            height: "18rem",
          }}
        />
        <VideoStream
          url={`/jai/calibrate/depth/videostream`}
          style={{
            width: "24rem",
            height: "18rem",
          }}
        />
      </HStack>
      {jaiStore.stereoMatrix && <CalibrationResultView />}
      <CalibrationMatrixView />
    </VStack>
  );
});

export const CalibrationPage = () => {
  useEffect(() => {
    jaiStore.fetchSubscribeJaiCalibration();

    return () => {
      jaiStore.fetchUnsubscribeJaiCalibration();
    };
  }, []);
  return (
    <PageRoot title="Calibration">
      <CalibrateView />
    </PageRoot>
  );
};
