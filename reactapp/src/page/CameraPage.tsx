import { Container, HStack, Text, VStack } from "@chakra-ui/react";
import { useEffect, useState } from "react";
import { subscribe } from "../connect/socket/subscribe";
import { CameraInfo } from "../data/CameraInfo";



const ControlPannel = () => {

    const [cameraStatus, setCameraStatus] = useState<CameraInfo>();

    useEffect(() => {
        return subscribe('/camera/info', [
            {
                eventName: 'camera_info',
                callback: (data: CameraInfo) => {
                    setCameraStatus(data);
                }
            }
        ]);
    }, []);

    return (<VStack style={{
        width: '50rem',
        height: '100%',
        background: 'black',
    }}>
        <Text>Camera Status</Text>
        {cameraStatus && (
            <>
                <Text>{cameraStatus?.height}</Text>
                <Text>{cameraStatus?.width}</Text>
                <Text>{cameraStatus?.distortion_model}</Text>
                <HStack style={{ width: '100%', justifyContent: 'flex-start' }}>
                    {[
                        {
                            key: 'd',
                            value: cameraStatus.d,
                            shape: [8, 1],
                        },
                        {
                            key: 'k',
                            value: cameraStatus.k,
                            shape: [3, 3],
                        },
                        {
                            key: 'r',
                            value: cameraStatus.r,
                            shape: [3, 3],
                        },
                        {
                            key: 'p',
                            value: cameraStatus.p,
                            shape: [3, 4],
                        },


                    ].map(({ key, value, shape }, index) => <VStack key={index} style={{ width: '100%', gap: 0, }}>
                        <Container style={{
                            width: '100%',
                            background: 'green',
                        }}>
                            <Text fontSize="lg" key={index}>{key}</Text>
                        </Container>

                        {
                            // 0 ... shape[0]
                            Array.from({ length: shape[0] }, (_, index) => index).map((i) => <HStack style={{
                                width: '15rem',
                                gap: 0,
                            }} key={i}>
                                {
                                    // 0 ... shape[1]
                                    Array.from({ length: shape[1] }, (_, index) => index).map(
                                        (j) => <Text key={j}
                                            fontSize="sm"
                                            style={{
                                                height: '2rem',
                                                flex: 1,
                                                border: '1px solid white',
                                                gap: 0,
                                            }}
                                        >{value[i * shape[1] + j].toFixed(4)}</Text>
                                    )
                                }
                            </HStack>)

                        }

                    </VStack>)
                    }
                </HStack>
            </>
        )}

    </VStack>)

};




const CameraPage = () => {
    return <HStack style={{
        width: '100vw',
        height: '100vh',

    }}>

        <VStack style={{
            height: '100%',
        }}>
            <Text>Camera Preview</Text>
            <img src="/preview_video_feed" alt="" />
        </VStack>
        <ControlPannel />
    </HStack>
};

export default CameraPage;