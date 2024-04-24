import {
  Button,
  Flex,
  HStack,
  IconButton,
  Input,
  Modal,
  ModalBody,
  ModalCloseButton,
  ModalContent,
  ModalHeader,
  ModalOverlay,
  useDisclosure,
} from "@chakra-ui/react";
import { useCallback, useState } from "react";
import { Btn } from "../../design/button/button";
import { httpPost } from "../../connect/http/request";
import { Storefront } from "@phosphor-icons/react/dist/ssr";
import { FloppyDisk } from "@phosphor-icons/react";

const MapSaveModal = (props: {
  isOpen: boolean;
  onClose: () => void;
  onSave: (name: string) => void;
}) => {
  const [name, setName] = useState<string>("");

  return (
    <Modal isOpen={props.isOpen} onClose={props.onClose}>
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>
          <ModalCloseButton />
        </ModalHeader>
        <ModalBody>
          <HStack>
            <Input
              placeholder="Map Name"
              value={name}
              onChange={(e) => setName(e.target.value)}
              onKeyPress={(e) => {
                if (e.key === "Enter") {
                  props.onSave(name);
                }
              }}
            />
            <IconButton
              icon={<FloppyDisk />}
              onClick={() => props.onSave(name)}
              aria-label=""
            />
          </HStack>
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};

const MapDuplicateModal = (props: {
  isOpen: boolean;
  onClose: () => void;
  onSave: () => void;
}) => {
  return (
    <Modal isOpen={props.isOpen} onClose={props.onClose}>
      <ModalOverlay />
      <ModalContent>
        <ModalBody>
          <Button onClick={props.onSave}>Overlap</Button>
          <Button onClick={props.onClose}>Cancel</Button>
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};

export const MapSaveBtn = (props: {}) => {
  const { isOpen, onOpen, onClose } = useDisclosure();
  const {
    isOpen: isDuplicateOpen,
    onOpen: onDuplicateOpen,
    onClose: onDuplicateClose,
  } = useDisclosure();

  const [name, setName] = useState<string>("");

  const save_map = useCallback((name: string, overwrite: boolean) => {
    if (name.length < 1) {
      return;
    }
    if (overwrite) onDuplicateClose();
    else onClose();
    httpPost("/slam/map/save", { filename: name, overwrite: overwrite })
      .onSuccess((data) => {
        if (data["message"] === "Filename unavailable" && !overwrite) {
          setName(name);
          onDuplicateOpen();
        }
      })
      .fetch();
  }, []);
  return (
    <Flex>
      <Btn onClick={onOpen}>Save</Btn>
      {isOpen && (
        <MapSaveModal
          isOpen={isOpen}
          onClose={onClose}
          onSave={(name) => {
            save_map(name, false);
          }}
        />
      )}
      {isDuplicateOpen && (
        <MapDuplicateModal
          isOpen={isDuplicateOpen}
          onClose={onDuplicateClose}
          onSave={() => {
            save_map(name, true);
          }}
        />
      )}
    </Flex>
  );
};
