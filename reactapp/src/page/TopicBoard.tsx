import { Checkbox, HStack, Switch, VStack } from "@chakra-ui/react"
import { Body3, H4 } from "../design/text/textsystem"
import { useEffect, useState } from "react"
import { rosSocket } from "../connect/socket/subscribe"
import { httpDel, httpGet, httpPost } from "../connect/http/request"




const TopicItem = ({ name, type, running }: { name: string, type?: string, running?: boolean }) => {


    const [hover, setHover] = useState(false)
    const [open, setOpen] = useState(running || false)

    const [log, setLog] = useState<string>('');

    const topicOpen = (status: Boolean) => {
        if (status) {
            httpPost(`/ros/topic`, {
                topic_name: name,
                topic_type: type
            }).then(response => {
                setOpen(true);
            });
        }
        else {
            httpPost(`/ros/topic/delete`, {
                topic_name: name
            }).then(response => {
                setOpen(false);
            });
        }
    }

    useEffect(() => {
        if (open) {
            console.log('subscribing to ', name)
            rosSocket.subscribe(name, (data: any) => {
                setLog(JSON.stringify(data));
            });
        }
        else {
            rosSocket.unsubscribe(name);
        }

    }, [open]);

    useEffect(() => {

    }, []);
    return <VStack style={{
        width: '100%'
    }}>
        <HStack style={{
            width: '100%',
            background: hover ? '#f0f0f0' : 'white',
            padding: '1rem',
        }}
            onMouseEnter={() => setHover(true)}
            onMouseLeave={() => setHover(false)}
        >
            <H4>{name}</H4>
            <H4>{type}</H4>
            <div style={{ flexGrow: 1 }}></div>
            <Switch isChecked={open} onChange={(e) => topicOpen(e.target.checked)} />
        </HStack>
        {
            open && (
                <VStack style={{
                    width: '100%',
                    padding: '1rem',
                    alignItems: 'flex-start'
                }}>
                    <HStack>
                        <Body3>{log}</Body3>
                        {
                            log.split("frame_id").length > 1 && <img src={
                                `
                            /ros/topic/preview/${JSON.parse(log)["frame_id"]}
                            `
                            } style={{ width: '100px', height: '100px' }} alt="" />
                        }
                    </HStack>

                </VStack>
            )
        }
        <div style={{
            width: '100%',
            height: '1px',
            background: '#1C1C1C0D'
        }}></div>
    </VStack>
}

const TopicBoard = () => {


    const [topicList, setTopicList] = useState<{ topic: string, type: string, running: boolean }[]>([]);

    useEffect(() => {
        httpGet("/ros/topic/list").then((data: { topic: string, type: string[], running: boolean }[]) => {
            setTopicList(
                data.map((topic) => {
                    return {
                        topic: topic.topic,
                        type: topic.type[0],
                        running: topic.running
                    }
                })
            );
        }).catch((e) => {
            console.log(e);
        })

            ;
    }, [])


    return <VStack style={{
        width: '100%',
        padding: '1rem',
    }}>
        {
            topicList.map((topic, i) => <TopicItem key={i} name={topic.topic} type={topic.type} running={topic.running} />)
        }

    </VStack>
}


export default TopicBoard;