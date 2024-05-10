import { HStack } from "@chakra-ui/react";
import { InfoCardBtn } from "../../design/other/infocard";
import { ArrowArcRight, House } from "@phosphor-icons/react";
import { httpPost } from "../../connect/http/request";

export const PolarizerControl = () => {
  const fetchTurnRight = () => {
    httpPost("/ell/angle", {
      angle: 45,
    }).fetch();
  };

  const fetchTurnHome = () => {
    httpPost("/ell/angle/home", {
      angle: 0,
    }).fetch();
  };

  return (
    <HStack>
      <InfoCardBtn title="HOME" Icon={House} onClick={fetchTurnHome} />
      <InfoCardBtn title="TURN" Icon={ArrowArcRight} onClick={fetchTurnRight} />
    </HStack>
  );
};
