import { observer } from "mobx-react-lite";
import { PageRoot } from "../../design/other/flexs";
import { lucidStore } from "../../stores/LucidStore";
import { Btn } from "../../design/button/button";
import { Body3, H4 } from "../../design/text/textsystem";
import {
  Switch,
  VStack,
  HStack,
  Box,
  Text,
  Badge,
  Progress,
  Divider,
  Card,
  CardBody,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  Grid,
  GridItem,
} from "@chakra-ui/react";
import { VideoStream } from "../../design/other/video";

const BrightnessMonitor = observer(
  ({
    latestAnalysis,
    latestProgress,
    finalVerification,
  }: {
    latestAnalysis: any;
    latestProgress: any;
    finalVerification: any;
  }) => {
    if (!lucidStore.hdrBurstMode) {
      return null;
    }

    return (
      <Box w="100%" p={4} border="1px" borderColor="gray.200" borderRadius="md">
        <H4 mb={3}>HDR Brightness Monitoring</H4>

        {/* Current Step Progress */}
        {latestProgress && (
          <Card mb={4}>
            <CardBody>
              <HStack justify="space-between" mb={2}>
                <VStack align="start" spacing={1}>
                  <Text fontWeight="bold">
                    Step {latestProgress.exposure_index + 1}/4:{" "}
                    {latestProgress.exposure_us}μs
                  </Text>
                  {latestProgress.exposure_retry &&
                    latestProgress.max_exposure_retries && (
                      <Text fontSize="sm" color="orange.600">
                        Capture Retry {latestProgress.exposure_retry}/
                        {latestProgress.max_exposure_retries}
                      </Text>
                    )}
                </VStack>
                <VStack align="end" spacing={1}>
                  <Badge
                    colorScheme={
                      latestProgress.exposure_valid ? "green" : "red"
                    }
                  >
                    {latestProgress.exposure_valid ? "✓ Valid" : "✗ Invalid"}
                  </Badge>
                  {latestProgress.critical_failure && (
                    <Badge colorScheme="red" variant="solid">
                      CRITICAL FAILURE
                    </Badge>
                  )}
                </VStack>
              </HStack>
              <Progress
                value={
                  (latestProgress.attempt / latestProgress.max_attempts) * 100
                }
                colorScheme={
                  latestProgress.exposure_valid
                    ? "green"
                    : latestProgress.critical_failure
                    ? "red"
                    : "orange"
                }
                mb={2}
              />
              <HStack justify="space-between" fontSize="sm" color="gray.600">
                <Text>
                  Setting attempt {latestProgress.attempt}/
                  {latestProgress.max_attempts}
                </Text>
                <Text>Cameras verified: {latestProgress.cameras_verified}</Text>
              </HStack>
              {latestProgress.error && (
                <Text
                  fontSize="sm"
                  color={
                    latestProgress.critical_failure ? "red.500" : "orange.500"
                  }
                  mt={2}
                >
                  {latestProgress.critical_failure ? "CRITICAL: " : "Warning: "}
                  {latestProgress.error}
                </Text>
              )}
            </CardBody>
          </Card>
        )}

        {/* Latest Brightness Analysis */}
        {latestAnalysis && (
          <Card mb={4}>
            <CardBody>
              <HStack justify="space-between" mb={3}>
                <VStack align="start" spacing={1}>
                  <Text fontWeight="bold">
                    Current Analysis: {latestAnalysis.exposure_us}μs
                  </Text>
                  <Text fontSize="sm" color="gray.600">
                    Step {latestAnalysis.exposure_index + 1}/4 •
                    {latestAnalysis.exposure_index === 0
                      ? "Darkest"
                      : latestAnalysis.exposure_index === 1
                      ? "Dark"
                      : latestAnalysis.exposure_index === 2
                      ? "Medium"
                      : "Bright"}{" "}
                    exposure
                  </Text>
                </VStack>
                <Badge
                  colorScheme={latestAnalysis.overall_valid ? "green" : "red"}
                  fontSize="md"
                >
                  {latestAnalysis.overall_valid ? "✓ Valid" : "✗ Invalid"}
                </Badge>
              </HStack>

              <Grid
                templateColumns="repeat(auto-fit, minmax(250px, 1fr))"
                gap={4}
              >
                {latestAnalysis &&
                  Object.entries(latestAnalysis.camera_results).map(
                    ([cameraName, result]: [string, any]) => (
                      <GridItem key={cameraName}>
                        <Box
                          p={4}
                          border="1px"
                          borderColor={result.valid ? "green.200" : "red.200"}
                          borderRadius="md"
                          bg={result.valid ? "green.50" : "red.50"}
                        >
                          <HStack justify="space-between" mb={3}>
                            <Text fontWeight="semibold" fontSize="sm">
                              {cameraName}
                            </Text>
                            <Badge
                              colorScheme={result.valid ? "green" : "red"}
                              size="sm"
                            >
                              {result.valid ? "✓" : "✗"}
                            </Badge>
                          </HStack>

                          <VStack align="start" spacing={2}>
                            <Stat size="sm">
                              <StatLabel>Brightness</StatLabel>
                              <StatNumber fontSize="lg">
                                {(result.mean * 100).toFixed(2)}%
                              </StatNumber>
                              <StatHelpText fontSize="xs">
                                σ: {(result.std * 100).toFixed(2)}% | median:{" "}
                                {(result.median * 100).toFixed(2)}%
                              </StatHelpText>
                            </Stat>

                            <Box w="100%">
                              <Text fontSize="xs" fontWeight="semibold" mb={1}>
                                Validation:
                              </Text>
                              <Text
                                fontSize="xs"
                                color={result.valid ? "green.700" : "red.700"}
                                fontStyle={result.valid ? "normal" : "italic"}
                                p={2}
                                bg={result.valid ? "green.100" : "red.100"}
                                borderRadius="md"
                              >
                                {result.reason}
                              </Text>
                            </Box>

                            <HStack
                              w="100%"
                              justify="space-between"
                              fontSize="xs"
                              color="gray.600"
                            >
                              <Text>Exposure: {result.exposure_us}μs</Text>
                              <Text>Step: {result.exposure_index + 1}/4</Text>
                            </HStack>
                          </VStack>
                        </Box>
                      </GridItem>
                    )
                  )}
              </Grid>
            </CardBody>
          </Card>
        )}

        {/* Final Verification Results */}
        {finalVerification && (
          <Card>
            <CardBody>
              <HStack justify="space-between" mb={3}>
                <Text fontWeight="bold">Final Verification</Text>
                <Badge
                  colorScheme={
                    finalVerification.overall_success ? "green" : "red"
                  }
                  fontSize="md"
                >
                  {finalVerification.overall_success ? "✓ SUCCESS" : "✗ FAILED"}
                </Badge>
              </HStack>

              {finalVerification &&
                Object.entries(finalVerification.camera_results).map(
                  ([cameraName, result]: [string, any]) => (
                    <Box
                      key={cameraName}
                      mb={4}
                      p={3}
                      border="1px"
                      borderColor="gray.100"
                      borderRadius="md"
                    >
                      <HStack justify="space-between" mb={2}>
                        <Text fontWeight="semibold">{cameraName}</Text>
                        <Badge colorScheme={result.is_valid ? "green" : "red"}>
                          {result.is_valid ? "✓ Valid" : "✗ Invalid"}
                        </Badge>
                      </HStack>

                      <Grid templateColumns="repeat(3, 1fr)" gap={2} mb={2}>
                        <Text fontSize="xs">
                          Monotonic: {result.is_monotonic ? "✓" : "✗"}
                        </Text>
                        <Text fontSize="xs">
                          Sufficient Δ:{" "}
                          {result.sufficient_increases ? "✓" : "✗"}
                        </Text>
                        <Text fontSize="xs">
                          Not Overexposed: {result.not_overexposed ? "✓" : "✗"}
                        </Text>
                      </Grid>

                      <HStack spacing={1} mb={1}>
                        <Text fontSize="xs" fontWeight="semibold">
                          Brightness:
                        </Text>
                        {result.brightness_values &&
                          result.brightness_values.map(
                            (brightness: number, i: number) => (
                              <Badge key={i} colorScheme="blue" fontSize="xs">
                                {(brightness * 100).toFixed(1)}%
                              </Badge>
                            )
                          )}
                      </HStack>

                      <HStack spacing={1}>
                        <Text fontSize="xs" fontWeight="semibold">
                          Increases:
                        </Text>
                        {result.increase_ratios &&
                          result.increase_ratios.map(
                            (ratio: number, i: number) => (
                              <Badge
                                key={i}
                                colorScheme={ratio >= 2.0 ? "green" : "orange"}
                                fontSize="xs"
                              >
                                {ratio.toFixed(1)}x
                              </Badge>
                            )
                          )}
                      </HStack>
                    </Box>
                  )
                )}
            </CardBody>
          </Card>
        )}

        {/* Analysis History Count */}
        <Box w="100%" mt={4}>
          <Text fontSize="sm" fontWeight="semibold" mb={2} color="blue.800">
            HDR Theory & Expected Values:
          </Text>
          <Grid templateColumns="repeat(4, 1fr)" gap={2} fontSize="xs" mb={3}>
            <VStack spacing={1} p={2} bg="gray.50" borderRadius="md">
              <Text fontWeight="semibold" color="blue.700">
                Step 1: 88μs
              </Text>
              <Text>Very Dark</Text>
              <Text color="gray.600">0.1%-10%</Text>
            </VStack>
            <VStack spacing={1} p={2} bg="gray.50" borderRadius="md">
              <Text fontWeight="semibold" color="blue.700">
                Step 2: 880μs
              </Text>
              <Text>10x brighter</Text>
              <Text color="gray.600">1%-50%</Text>
            </VStack>
            <VStack spacing={1} p={2} bg="gray.50" borderRadius="md">
              <Text fontWeight="semibold" color="blue.700">
                Step 3: 8.8ms
              </Text>
              <Text>10x brighter</Text>
              <Text color="gray.600">10%-80%</Text>
            </VStack>
            <VStack spacing={1} p={2} bg="gray.50" borderRadius="md">
              <Text fontWeight="semibold" color="blue.700">
                Step 4: 88ms
              </Text>
              <Text>10x brighter</Text>
              <Text color="gray.600">50%-95%</Text>
            </VStack>
          </Grid>
          <Text fontSize="xs" color="gray.600" mb={3}>
            * Actual values depend on scene lighting conditions. System
            validates relative increases between steps.
          </Text>

          <HStack spacing={4} fontSize="xs" color="gray.600">
            <Text>
              Total analyses: {lucidStore.brightnessAnalysisList.length}
            </Text>
            <Text>Step progress: {lucidStore.hdrStepProgressList.length}</Text>
            <Text>
              Last update:{" "}
              {latestAnalysis
                ? new Date(latestAnalysis.timestamp * 1000).toLocaleTimeString()
                : "Never"}
            </Text>
          </HStack>
        </Box>
      </Box>
    );
  }
);

export const LucidCaptureView = observer(() => {
  const latestAnalysis = lucidStore.getLatestBrightnessAnalysis();
  const latestProgress = lucidStore.getLatestStepProgress();
  const finalVerification = lucidStore.hdrFinalVerification;

  return (
    <PageRoot title="Lucid">
      {/* Storage Controls */}
      <HStack mb={4}>
        {lucidStore.storageEnabled ? (
          <Btn onClick={lucidStore.fetchDisableStorage}>Disable Storage</Btn>
        ) : (
          <Btn onClick={lucidStore.fetchEnableStorage}>Enable Storage</Btn>
        )}
        {lucidStore.lucidStatus.hasOwnProperty("single_storage_mode") && (
          <HStack>
            <Text>Single Storage Mode:</Text>
            <Switch
              isChecked={lucidStore.lucidStatus["single_storage_mode"]}
              onChange={(e) => {
                lucidStore.fetchUpdateStatusAttributes({
                  single_storage_mode: e.target.checked,
                });
              }}
            />
          </HStack>
        )}
      </HStack>

      {/* HDR Burst Controls */}
      <HStack mb={4}>
        {lucidStore.hdrBurstMode ? (
          <Btn onClick={lucidStore.fetchDisableHDRBurst} color="red">
            Disable HDR Burst
          </Btn>
        ) : (
          <Btn onClick={lucidStore.fetchEnableHDRBurst} color="blue">
            Enable HDR Burst
          </Btn>
        )}

        {lucidStore.hdrBurstMode && (
          <Btn onClick={lucidStore.fetchCaptureHDRBurst} color="green">
            {lucidStore.hdrBurstInProgress
              ? "Capturing..."
              : "Capture HDR Burst"}
          </Btn>
        )}

        <Btn onClick={lucidStore.clearBrightnessData} color="gray" size="sm">
          Clear Logs
        </Btn>
      </HStack>

      {/* HDR Status Display */}
      {lucidStore.hdrBurstMode && (
        <Card mb={4} bg="blue.50" borderColor="blue.200">
          <CardBody>
            <HStack mb={3}>
              <Badge colorScheme="blue" fontSize="md">
                HDR Mode Active
              </Badge>
              {lucidStore.hdrBurstInProgress && (
                <Badge colorScheme="orange" fontSize="md">
                  In Progress
                </Badge>
              )}
            </HStack>

            <VStack align="start" spacing={2}>
              <Text fontSize="sm" fontWeight="semibold">
                Current Step: {lucidStore.hdrBurstCurrentStep || "Idle"}
              </Text>

              {/* Exposure Timeline */}
              <Box w="100%">
                <Text
                  fontSize="xs"
                  fontWeight="semibold"
                  mb={2}
                  color="gray.700"
                >
                  HDR Exposure Sequence:
                </Text>
                <HStack spacing={2}>
                  {[
                    { us: 88, label: "88μs", desc: "Very Dark" },
                    { us: 880, label: "880μs", desc: "Dark" },
                    { us: 8800, label: "8.8ms", desc: "Medium" },
                    { us: 88000, label: "88ms", desc: "Bright" },
                  ].map((exposure, index) => {
                    const isCurrent = lucidStore.hdrBurstCurrentStep.includes(
                      `${exposure.us}us`
                    );
                    const isCompleted =
                      latestAnalysis && latestAnalysis.exposure_index > index;
                    const isValid = isCompleted && latestAnalysis.overall_valid;

                    return (
                      <VStack key={index} spacing={1} align="center">
                        <Badge
                          colorScheme={
                            isCurrent
                              ? "orange"
                              : isCompleted
                              ? isValid
                                ? "green"
                                : "red"
                              : "gray"
                          }
                          variant={isCurrent ? "solid" : "outline"}
                          fontSize="xs"
                        >
                          {exposure.label}
                        </Badge>
                        <Text fontSize="xs" color="gray.600" textAlign="center">
                          {exposure.desc}
                        </Text>
                        {isCurrent && (
                          <Text
                            fontSize="xs"
                            color="orange.600"
                            fontWeight="bold"
                          >
                            Current
                          </Text>
                        )}
                        {isCompleted && (
                          <Text
                            fontSize="xs"
                            color={isValid ? "green.600" : "red.600"}
                          >
                            {isValid ? "✓" : "✗"}
                          </Text>
                        )}
                      </VStack>
                    );
                  })}
                </HStack>
              </Box>

              {/* Overall Progress */}
              {latestAnalysis && (
                <Box w="100%">
                  <Text fontSize="xs" fontWeight="semibold" mb={1}>
                    Overall Progress:
                  </Text>
                  <Progress
                    value={((latestAnalysis.exposure_index + 1) / 4) * 100}
                    colorScheme={
                      latestAnalysis.overall_valid ? "green" : "orange"
                    }
                    size="sm"
                    borderRadius="md"
                  />
                  <Text fontSize="xs" color="gray.600" mt={1}>
                    Step {latestAnalysis.exposure_index + 1} of 4 completed
                  </Text>
                </Box>
              )}
            </VStack>
          </CardBody>
        </Card>
      )}

      {/* Brightness Monitoring */}
      <BrightnessMonitor
        latestAnalysis={latestAnalysis}
        latestProgress={latestProgress}
        finalVerification={finalVerification}
      />

      <Divider my={4} />

      {/* Status and Video Stream */}
      <VStack
        style={{
          width: "100%",
          height: "100vh",
        }}
      >
        {lucidStore.lucidStatusJson.split(",").map((v, i) => (
          <Body3
            key={i}
            style={{
              width: "100%",
            }}
          >
            {"," + v}
          </Body3>
        ))}
        <VideoStream
          url={"/lucid/stream/preview"}
          style={{ width: "100%", height: "auto" }}
        />
      </VStack>
    </PageRoot>
  );
});
