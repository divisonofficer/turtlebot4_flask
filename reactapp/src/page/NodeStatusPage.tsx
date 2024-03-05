import { useEffect, useState } from "react";
import { httpGet } from "../connect/http/request";
import { Text, VStack } from "@chakra-ui/react";
import { useLoaderData } from "react-router-dom";

const NodeStatusPage = () => {
    const { pkg, node } = useLoaderData() as { pkg: string, node: string };

    const [nodeLogs, setNodeLogs] = useState<string[]>([]);

    const fetchLogs = () => {
        httpGet(`/ros/node/${pkg}/${node}`).then((data: { logs: string }) => {
            setNodeLogs(data.logs.split('\n'));
        });
    }

    useEffect(() => {
        const intervel = setInterval(fetchLogs, 1000);

        return () => {
            clearInterval(intervel);
        }
    }, []);

    return <VStack style={{
        width: '100vw',
        height: '100vh',
    }}>
        <Text>{pkg}_{node}</Text>
        <VStack style={{
            width: '100vw',
            flexGrow: 1,
            overflowY: 'auto',
            justifyContent: 'flex-start',
            alignItems: 'flex-start',
        }}>
            {nodeLogs.map((log, index) => <Text key={index} fontSize='sm'>{log}</Text>)}
        </VStack>
    </VStack>
}

export default NodeStatusPage;