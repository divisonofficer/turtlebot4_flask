import {
  Flex,
  Modal,
  ModalBody,
  ModalCloseButton,
  ModalContent,
  ModalHeader,
  ModalOverlay,
} from "@chakra-ui/react";
import { observer } from "mobx-react";
import { alertStore } from "../../stores/AlertStore";
import { Body1, H1 } from "../../design/text/textsystem";

export const AlertView = observer(() => {
  const isOpen = alertStore.alertList.length > 0;
  const onClose = () => {
    alertStore.consumeFirstAlert();
  };

  const alertData = isOpen ? alertStore.alertList[0] : undefined;

  return (
    <Flex>
      {isOpen && (
        <Modal isOpen={isOpen} onClose={onClose}>
          <ModalOverlay />
          <ModalContent>
            <ModalHeader>
              {alertData?.type}
              <ModalCloseButton />
            </ModalHeader>
            <ModalBody>
              <H1> {alertData?.title}</H1>
              <Body1>{alertData?.message}</Body1>
            </ModalBody>
          </ModalContent>
        </Modal>
      )}
    </Flex>
  );
});
