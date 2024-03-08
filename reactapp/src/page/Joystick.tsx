import { ArrowRightIcon } from "@chakra-ui/icons"
import { Container, HStack, VStack } from "@chakra-ui/react"
import { useEffect, useState } from "react";
import { rosSocket } from "../connect/socket/subscribe";




const stickProfile = [
    {
        arrow: 225,
        d: 1,
        angle: 1,
    },
    {
        arrow: 270,
        d: 1,
        angle: 0,
    },
    {
        arrow: 315,
        d: 1,
        angle: -1,
    },
    {
        arrow: 180,
        d: 0,
        angle: 1,
    },
    {
        arrow: 270,
        d: 0,
        angle: 0,
    },
    {
        arrow: 0,
        d: 0,
        angle: -1,
    },

    {
        arrow: 135,
        d: -1,
        angle: 1,
    },
    {
        arrow: 90,
        d: -1,
        angle: 0,
    },
    {
        arrow: 45,
        d: -1,
        angle: -1,
    },
]

const JoysticPage = () => {


    const [intervalJob, setIntervalJob] = useState<NodeJS.Timeout>();

    const handleButtonDown = (index: number) => {
        console.log('down', index);
        const job = setInterval(() => {
            rosSocket.publish('/drive', {
                linear: {
                    x: stickProfile[index].d,
                    y: 0,
                    z: 0,
                },
                angular: {
                    x: 0,
                    y: 0,
                    z: stickProfile[index].angle,
                }
            })
        }, 100);
        setIntervalJob(job);
    };

    const handleButtonUp = (index: number) => {
        console.log('up', index);
        if (intervalJob) {
            clearInterval(intervalJob);
            setIntervalJob(undefined);
        }
    }

    useEffect(() => {
        return () => {
            if (intervalJob) {
                clearInterval(intervalJob);
                setIntervalJob(undefined);
            }
        }
    }, [intervalJob]);


    const JoyStick = () => {
        return <VStack>
            {
                [0, 1, 2].map((index) =>
                    <HStack>
                        {[0, 1, 2].map((hindex) => <Container style={{
                            width: '5rem',
                            height: '5rem',
                            borderRadius: '50%',
                            background: '#D7F9FB',
                        }}
                            onMouseDown={() => handleButtonDown(index * 3 + hindex)}
                            onMouseUp={() => handleButtonUp(index * 3 + hindex)}
                        >
                            <ArrowRightIcon style={{
                                transform: `rotate(${stickProfile[index * 3 + hindex].arrow}deg)`
                            }} />
                        </Container>)}
                    </HStack>)
            }
        </VStack>
    }

    return <VStack style={{
        width: '100%',
    }}>
        <JoyStick />
    </VStack>
}
export default JoysticPage;