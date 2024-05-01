import {
  Flex,
  FlexProps,
  HStack,
  Modal,
  ModalBody,
  ModalCloseButton,
  ModalContent,
  ModalHeader,
  ModalOverlay,
  StackProps,
  VStack,
} from "@chakra-ui/react";
import { Body3 } from "../../design/text/textsystem";
import { CaptureSpace, captureStore } from "../../stores/CaptureStore";

const SpaceItem = (props: StackProps & { space: CaptureSpace }) => {
  const { space } = props;
  return (
    <HStack
      {...props}
      style={{
        ...props.style,
        borderRadius: "0.5rem",

        border: "1px solid #ececec",
        padding: "0.5rem",
      }}
      _hover={{
        background: "#acacac",
        cursor: "pointer",
      }}
    >
      <img
        src={`/slam/${space.map_name}/image`}
        alt="Map undefiend"
        style={{
          width: "6rem",
          height: "6rem",
          borderRadius: "0.5rem",
          background: "#ececec",
        }}
      />

      <VStack alignItems="flex-start" width="8rem">
        <Body3>ID : {space.space_id}</Body3>
        <Body3>Captures : {space.captures.length}</Body3>
      </VStack>
    </HStack>
  );
};

export const CaptureSpaceModal = (props: {
  isOpen: boolean;
  onClose: () => void;
}) => {
  return (
    <Modal isOpen={props.isOpen} onClose={props.onClose}>
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>
          Launch a space
          <ModalCloseButton />
        </ModalHeader>
        <ModalBody>
          <VStack>
            <Flex wrap={"wrap"}>
              {captureStore.captureSpaceList.map((space, index) => (
                <SpaceItem
                  space={space}
                  key={index}
                  onClick={() => {
                    captureStore.loadSpace(space);
                    props.onClose();
                  }}
                />
              ))}
            </Flex>
          </VStack>
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};
