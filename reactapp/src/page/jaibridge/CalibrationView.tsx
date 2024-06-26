import {
  Flex,
  HStack,
  IconButton,
  Img,
  Table,
  Tbody,
  Td,
  Tr,
  VStack,
} from "@chakra-ui/react";
import { observer } from "mobx-react";
import { VideoStream } from "../../design/other/video";
import { jaiStore } from "../../stores/JaiStore";
import { Body3, H2, H3 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";
import { PageRoot } from "../../design/other/flexs";
import { useEffect, useState } from "react";
import { Trash } from "@phosphor-icons/react";

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

const CalibrationResultView = observer(() => {
  const ImagePairListView = (props: { imageCount: number }) => {
    return (
      <VStack
        style={{
          width: "24rem",
          maxHeight: "96rem",
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
  };

  const [vIdx, setVIdx] = useState(0);

  const ReprojectionErrorGraph = (props: {
    vIdx: number;
    errorLeft: number[];
    errorRight: number[];
  }) => {
    const errorMax = Math.max(...props.errorLeft, ...props.errorRight);
    const errorMin = 0;
    const length = props.errorLeft.length;

    const graphVGap = errorMax > 1 ? 1 : 0.1;
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
          }}
        >
          <VStack gap={0} height="100%">
            {Array.from(Array(graphVGapCount)).map((_, i) => (
              <Flex
                style={{
                  height: `${100 / (errorMax / graphVGap)}%`,
                  flexDirection: "column",
                  justifyContent: "flex-end",
                }}
              >
                <Body3>
                  {(graphVGap * (graphVGapCount - 1 - i)).toFixed(1)}
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
                      ((props.errorLeft[i] - errorMin) /
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
              src={`/jai/calibrate/chessboard/${vIdx}/0`}
              style={{ width: "50%" }}
            />
            <Img
              src={`/jai/calibrate/chessboard/${vIdx}/1`}
              style={{ width: "50%" }}
            />
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

export const CalibrateView = observer(() => {
  return (
    <VStack>
      <HStack>
        <Btn size="sm" onClick={() => jaiStore.calibrationCapture()}>
          Capture
        </Btn>
        <Btn size="sm" onClick={() => jaiStore.fetchCalibrationSave()}>
          Save
        </Btn>
      </HStack>
      <HStack>
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
