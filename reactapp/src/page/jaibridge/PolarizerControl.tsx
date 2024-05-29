import { HStack, Input, VStack } from "@chakra-ui/react";
import { InfoCardBtn } from "../../design/other/infocard";
import { ArrowArcRight, House } from "@phosphor-icons/react";
import { httpPost } from "../../connect/http/request";
import { H2 } from "../../design/text/textsystem";
import { useState } from "react";

export const PolarizerControl = () => {
  const fetchTurnRight = () => {
    httpPost("/ell/angle", {
      angle: angle,
    }).fetch();
  };

  const fetchTurnHome = () => {
    httpPost("/ell/angle/home", {
      angle: 0,
    }).fetch();
  };

  const [angle, setAngle] = useState(45);

  return (
    <VStack>
      <H2>Polarizer Control</H2>
      <HStack>
        <InfoCardBtn title="HOME" Icon={House} onClick={fetchTurnHome} />
        <InfoCardBtn
          title="TURN"
          Icon={ArrowArcRight}
          onClick={fetchTurnRight}
        />
        <Input
          placeholder="angle"
          type="number"
          value={angle}
          width="5rem"
          onChange={(e) => setAngle(Number(e.target.value))}
        />
      </HStack>{" "}
    </VStack>
  );
};
