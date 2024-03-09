import { Flex, FlexProps, useMediaQuery } from "@chakra-ui/react";

const VStackAdj = (props: FlexProps) => {
  const isMobile = useMediaQuery("(max-width: 600px)")[0];

  return <Flex direction={isMobile ? "column" : "row"} {...props} />;
};
export { VStackAdj };
