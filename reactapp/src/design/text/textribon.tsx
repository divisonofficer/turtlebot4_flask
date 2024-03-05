import { Text } from "@chakra-ui/react";



type props = {
    children: React.ReactNode;
    style?: React.CSSProperties;
    size: "lg" | "md" | "sm";
    color?: string;
    textColor?: string;
}


const TextRibon = ({
    children, style, size, color, textColor
}: props) => (
    <Text fontSize={size} fontWeight="bold" textAlign="left" style={{
        background: color || '#8C8C8C66',
        padding: '0.2rem 0.5rem',
        borderRadius: '1rem',
        color: textColor || 'white',
        ...style
    }}>
        {children}
    </Text>
);


export {
    TextRibon
}