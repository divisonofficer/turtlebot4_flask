import { Grid, VStack } from "@chakra-ui/layout";
import { PageRoot } from "../../design/other/flexs";
import { useEffect, useState } from "react";
import { httpGet, httpPost } from "../../connect/http/request";
import { RosService } from "../../data/Service";
import { Body3, H3, H4 } from "../../design/text/textsystem";
import {
  Modal,
  ModalBody,
  ModalCloseButton,
  ModalContent,
  ModalFooter,
  ModalHeader,
  ModalOverlay,
} from "@chakra-ui/modal";
import { useDisclosure } from "@chakra-ui/hooks";
import { Button } from "@chakra-ui/button";
import { Btn } from "../../design/button/button";
import { Textarea } from "@chakra-ui/textarea";

const ServiceCallModal = ({
  isOpen,
  close,
  service,
}: {
  isOpen: boolean;
  close: () => void;
  service: RosService;
}) => {
  const [jsonBody, setJsonBody] = useState<string>("{}");
  const [responseBody, setResponseBody] = useState<string>("{}");

  const getJsonFormat = () => {
    httpPost("/ros/topic/type/format", {
      topic_type: service.type,
    })
      .onSuccess((data: any) => {
        console.log(data);
      })
      .onError((code, message, e) => {})
      .fetch();
  };

  const fetchJson = () => {
    httpPost("/ros/service/call", {
      service_name: service.service,
      service_type: service.type,
      request_data: JSON.parse(jsonBody),
    })
      .onSuccess((data: any) => {
        setResponseBody(JSON.stringify(data, null, 2));
      })
      .onError((code, message, e) => {
        setResponseBody("Error: " + message + " " + code);
      })
      .fetch();
  };

  return (
    <Modal isOpen={isOpen} onClose={close}>
      <ModalOverlay />
      <ModalContent>
        <ModalHeader>{service.service}</ModalHeader>
        <ModalCloseButton />
        <ModalBody>
          <H3>Request Body</H3>
          <Textarea
            value={jsonBody}
            onChange={(e) => setJsonBody(e.target.value)}
          />

          <H3>Response</H3>
          <Textarea value={responseBody} isReadOnly />
        </ModalBody>

        <ModalFooter gap="5">
          <Btn onClick={close}>Close</Btn>
          <Btn onClick={fetchJson}>Send</Btn>
          <Btn
            onClick={() => {
              getJsonFormat();
            }}
          >
            Refresh
          </Btn>
        </ModalFooter>
      </ModalContent>
    </Modal>
  );
};

const ServiceCallItem = ({ service }: { service: RosService }) => {
  const nm = service.service.split("/");
  const lastBlock = nm[nm.length - 1];
  const { isOpen, onOpen, onClose } = useDisclosure();

  const pkgNS = nm.slice(0, nm.length - 1).join("/");

  return (
    <VStack
      style={{
        width: "15rem",
        height: "8rem",
        padding: "1.5rem 1.2rem",
        borderRadius: "1rem",
        background: "#E3F5FF",
        alignItems: "flex-start",
      }}
      onClick={onOpen}
    >
      <H4>{lastBlock}</H4>
      <Body3>{pkgNS}</Body3>
      <ServiceCallModal isOpen={isOpen} close={onClose} service={service} />
    </VStack>
  );
};

const ServiceCallList = () => {
  const [services, setServices] = useState<RosService[]>([]);

  useEffect(() => {
    // fetch services
    httpGet("/ros/service/list")
      .onSuccess((data: RosService[]) => {
        setServices(data);
      })
      .fetch();
  }, []);

  return (
    <Grid templateColumns="repeat(4, 1fr)" gap="1rem">
      {services.map((service, index) => (
        <ServiceCallItem key={index} service={service} />
      ))}
    </Grid>
  );
};

const ServiceCallPage = () => {
  return (
    <PageRoot title="Service Call">
      <ServiceCallList />
    </PageRoot>
  );
};

export default ServiceCallPage;
