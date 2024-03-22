import React, { ReactNode } from "react";
import { Text, TextProps } from "@chakra-ui/react";

type Props = TextProps & {
  children: ReactNode;
};

const H1 = (props: Props) => (
  <Text fontSize="4xl" fontWeight="bold" textAlign="left" {...props}>
    {props.children}
  </Text>
);

const H2 = (props: Props) => (
  <Text fontSize="3xl" fontWeight="bold" textAlign="left" {...props}>
    {props.children}
  </Text>
);

const H3 = (props: Props) => (
  <Text fontSize="2xl" fontWeight="bold" textAlign="left" {...props}>
    {props.children}
  </Text>
);

const H4 = (props: Props) => (
  <Text fontSize="xl" fontWeight="bold" textAlign="left" {...props}>
    {props.children}
  </Text>
);

const Body1 = (props: Props) => (
  <Text fontSize="lg" textAlign="left" {...props}>
    {props.children}
  </Text>
);

const Body2 = (props: Props) => (
  <Text fontSize="md" textAlign="left" {...props}>
    {props.children}
  </Text>
);

const Body3 = (props: Props) => (
  <Text fontSize="sm" textAlign="left" {...props}>
    {props.children}
  </Text>
);

export { H1, H2, H3, H4, Body1, Body2, Body3 };
