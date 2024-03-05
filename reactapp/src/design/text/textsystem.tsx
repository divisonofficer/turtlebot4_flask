import React, { ReactNode } from "react";
import { Text } from "@chakra-ui/react";

type Props = {
    children: ReactNode;
    style?: React.CSSProperties;
};

const H1 = ({
    children, style
}: Props) => (
    <Text fontSize="4xl" fontWeight="bold" textAlign="left" style={style}>
        {children}
    </Text>
);

const H2 = ({
    children, style
}: Props) => (
    <Text fontSize="3xl" fontWeight="bold" textAlign="left" style={style}>
        {children}
    </Text>
);

const H3 = ({
    children, style
}: Props) => (
    <Text fontSize="2xl" fontWeight="bold" textAlign="left" style={style}>
        {children}
    </Text>
);

const H4 = ({
    children, style
}: Props) => (
    <Text fontSize="xl" fontWeight="bold" textAlign="left" style={style}>
        {children}
    </Text>
);

const Body1 = ({
    children, style
}: Props) => (
    <Text fontSize="lg" textAlign="left" style={style}>
        {children}
    </Text>
);

const Body2 = ({
    children, style
}: Props) => (
    <Text fontSize="md" textAlign="left" style={style}>
        {children}
    </Text>
);

const Body3 = ({
    children, style
}: Props) => (
    <Text fontSize="sm" textAlign="left" style={style}>
        {children}
    </Text>
);

export {
    H1,
    H2,
    H3,
    H4,
    Body1,
    Body2,
    Body3,
}
