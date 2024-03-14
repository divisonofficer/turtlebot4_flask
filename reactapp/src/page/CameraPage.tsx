import { Container, HStack, Radio, RadioGroup, Text, VStack } from "@chakra-ui/react";
import { useEffect, useRef, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";
import { CameraInfo } from "../data/CameraInfo";
import ServiceStatusBar from "./ServiceStatusBar";
import PkgStatusBar from "./PkgStatusBar";



const ControlPannel = () => {

    const [cameraStatus, setCameraStatus] = useState<CameraInfo>();
    const [ip, setIp] = useState("");
    useEffect(() => {
        // rosSocket.subscribe('camera_info', (data: CameraInfo) => {
        //     setCameraStatus(data);
        // });

        rosSocket.subscribe('/ip', (data: { data: string }) => {
            setIp(data.data);
        });
    }, []);

    return (<VStack style={{
        width: '50rem',
        height: '100%',
        background: 'black',
    }}>
        <Text>Camera Status</Text>
        <Text>IP: {ip}</Text>
        {cameraStatus && (
            <>
                <Text>{cameraStatus?.height}</Text>
                <Text>{cameraStatus?.width}</Text>
                <Text>{cameraStatus?.distortion_model}</Text>
                <VStack style={{ width: '100%', justifyContent: 'flex-start' }}>
                    {[
                        {
                            key: 'd',
                            value: cameraStatus.d,
                            shape: [cameraStatus.d.length, 1],
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
                            <Text fontSize="lg" key={index}>{key} {value.length} items</Text>
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
                </VStack>
            </>
        )}

    </VStack>)

};


type PreviewSource = "/ros/camera/preview" | "/ros/camera/color";



interface Detection {
    label: string;
    score: string;
    bbox: [number, number, number, number];
}


const CameraDetectionCanvas = ({ source }: { source: string }) => {
    const [detections, setDetections] = useState<Detection[]>([]);
    const containerRef = useRef<HTMLDivElement>(null);
    const [containerSize, setContainerSize] = useState({ width: 0, height: 0 });

    const [w, h] = [300, 300];
    useEffect(() => {
        rosSocket.subscribe("camera_detection", (data) => {
            if (data)
                setDetections(data);
        });

        const updateSize = () => {
            if (containerRef.current) {
                setContainerSize({
                    width: containerRef.current.offsetWidth,
                    height: containerRef.current.offsetHeight
                });
            }
        };

        window.addEventListener('resize', updateSize);
        updateSize();

        return () => window.removeEventListener('resize', updateSize);
    }, []);

    return (
        <VStack ref={containerRef} style={{
            width: '100%',
            height: '100%',
            overflow: 'hidden',
        }}>

            <VStack style={{
                width: '100%',
                height: '100%',
                backgroundImage: `url(${source})`,
                backgroundSize: 'contain',
                backgroundPosition: 'center',
                backgroundRepeat: 'no-repeat',
                position: 'relative',
            }}>
                {detections.map((detection, index) => {
                    const scaleX = containerSize.width / w; // Adjust these values
                    const scaleY = containerSize.height / h; // Adjust these values
                    const left = detection.bbox[0] * scaleX;
                    const top = detection.bbox[1] * scaleY;
                    const width = (detection.bbox[2] - detection.bbox[0]) * scaleX;
                    const height = (detection.bbox[3] - detection.bbox[1]) * scaleY;

                    return (
                        <div key={index} style={{
                            position: 'absolute',
                            left: `${left}px`,
                            top: `${top}px`,
                            width: `${width}px`,
                            height: `${height}px`,
                            border: '2px solid red',
                        }}>
                            <Text fontSize="sm">{detection.label}</Text>


                        </div>
                    );
                })}
            </VStack>
        </VStack>
    );
}



const CameraPage = () => {
    const [previewSource, setPreviewSource] = useState<PreviewSource>("/ros/camera/preview");
    return <VStack style={{
        width: '100vw',
        height: '100vh',

    }}>
        <ServiceStatusBar />
        <PkgStatusBar />
        <HStack style={{
            width: '100vw',
            flexGrow: 1,
        }}>

            <VStack style={{
                height: '100%',
            }}>
                <Text>Camera Preview</Text>
                <Container style={{
                    width: '100rem',
                    height: '100rem',
                }}>
                    <CameraDetectionCanvas source={previewSource} />
                </Container>

                <RadioGroup onChange={(e) => setPreviewSource(e as PreviewSource)} value={previewSource}>
                    <Radio value="/ros/camera/preview">Preview</Radio>
                    <Radio value="/ros/camera/color">Color</Radio>
                </RadioGroup>
            </VStack>
            <ControlPannel />
        </HStack>
    </VStack>

};

export default CameraPage;