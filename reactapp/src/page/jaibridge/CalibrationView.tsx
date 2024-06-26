import { Flex, HStack, Table, Tbody, Td, Tr, VStack } from "@chakra-ui/react";
import { observer } from "mobx-react";
import { VideoStream } from "../../design/other/video";
import { jaiStore } from "../../stores/JaiStore";
import { H3 } from "../../design/text/textsystem";
import { Btn } from "../../design/button/button";

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

export const CalibrateView = () => {
  return (
    <VStack>
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
      <CalibrationMatrixView />
    </VStack>
  );
};
