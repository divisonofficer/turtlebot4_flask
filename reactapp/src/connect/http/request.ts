class RequestManager{




    request = async (url: string, method: string, data: any) => {
        const response = await fetch(url, {
            method: method,
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify(data)
        });

        return await response.json();
    }



}


const requestManager = new RequestManager();

const httpPost = async (url: string, data: any = undefined) => {
    return await requestManager.request(url, 'POST', data);
}

const httpGet = async (url: string) => {
    return await requestManager.request(url, 'GET', undefined);
}

const httpDel = async (url: string) => {
    return await requestManager.request(url, 'DELETE', undefined);
}

export {
    httpPost,
    httpGet,
    httpDel
}



