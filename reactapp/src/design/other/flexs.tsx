import {
  Divider,
  Flex,
  FlexProps,
  HStack,
  VStack,
  useMediaQuery,
} from "@chakra-ui/react";
import { H1, H2 } from "../text/textsystem";
import { Color } from "../color";
import { useNavigate } from "react-router-dom";
import { Btn } from "../button/button";

const VStackAdj = (props: FlexProps) => {
  const isMobile = useMediaQuery("(max-width: 600px)")[0];

  return <Flex direction={isMobile ? "column" : "row"} {...props} />;
};
export { VStackAdj };

export const PageRoot = (
  props: FlexProps & {
    title: string;
    color?: string;
    backPage?: {
      title: string;
      path: string;
    };
  }
) => {
  const navigate = useNavigate();
  return (
    <Flex
      direction="column"
      style={{
        ...props.style,
        width: "100%",
        height: "100vh",
        alignItems: "flex-start",
        justifyContent: "flex-start",
      }}
    >
      <HStack>
        {props.backPage && (
          <Btn
            varient="borderless"
            size="md"
            onClick={() => navigate(props.backPage!.path)}
          >
            {props.backPage.title + " <"}
          </Btn>
        )}
        <H1
          style={{
            margin: "1rem",
          }}
        >
          {props.title}
        </H1>
      </HStack>

      <Divider />

      {props.children}
    </Flex>
  );
};
