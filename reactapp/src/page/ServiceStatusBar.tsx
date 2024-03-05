import { Container, HStack, Text } from "@chakra-ui/react"
import { useEffect, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";

const ServiceStatusBar = () => {



    const [serviceStatus, setServiceStatus] = useState<{ [key: string]: boolean }>({});


    useEffect(() => {
        rosSocket.subscribe('/status_monitoring', (data: { [key: string]: boolean }) => {
            setServiceStatus(data);
        });
    }, []);

    return (
        <HStack style={{
            width: '100vw',
            height: '20rem',
            justifyContent: 'flex-start',
            paddingLeft: '2rem',
            background: '#982950'
        }}>
            {serviceStatus && Object.keys(serviceStatus).map((key, index) => (
                <HStack key={index} style={{}}>
                    <Text>{key}</Text>
                    <Container style={{
                        marginLeft: '1rem',
                        marginRight: '3rem',
                        width: '2rem',
                        height: '2rem',
                        borderRadius: '50%',
                        background: serviceStatus[key] ? '#00ff00' : '#ff0000',
                    }} />
                </HStack>
            ))}
        </HStack>
    );
}

export default ServiceStatusBar;