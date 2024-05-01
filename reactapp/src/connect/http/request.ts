const DEBUG = true;

class NotSuccessResponseError extends Error {
  httpCode: number;
  body?: string;
  constructor(httpCode: number, body?: string) {
    super(`NotSuccessResponseError: ${httpCode} ${body}`);
    this.name = "NotSuccessResponseError";
    this.httpCode = httpCode;
    this.body = body;
  }
}

class RequestManager {
  request = async (url: string, method: string, data: any) => {
    if (DEBUG) {
      console.log("RequestManager.request", url, method, data);
    }

    const fetchPromise = fetch(url, {
      method: method,
      headers: {
        "Content-Type": "application/json",
        "Access-Control-Allow-Origin": "*",
        "Strict-Origin-Policy": "false",
      },
      body: JSON.stringify(data),
    });

    let response;

    try {
      response = await fetchPromise;
    } catch (e) {
      console.log(e);
      throw new Error(`RequestManager.request: ${e}`);
    }
    if (DEBUG) {
      console.log("RequestManager.request", url, response.status);
    } // 200이 아닌 경우
    if (response.status !== 200) {
      try {
        throw new NotSuccessResponseError(
          response.status,
          await response.text()
        );
      } catch (e) {
        throw new Error(
          `RequestManager.request: ${response.status} ${response.statusText}`
        );
      }
    }

    // 200인데 empty인 경우
    if (
      response.status === 200 &&
      response.headers.get("content-length") === "0"
    ) {
      return {};
    }

    try {
      return await response.json();
    } catch (e) {
      return await response.text();
    }
  };
}

interface ICall {
  url: string;
  method: "POST" | "GET" | "DELETE" | "PUT";
  data: any;
  callbackOnSuccess: (body: any) => void;
  callbackOnError: (
    code: number | undefined,
    message: string | undefined,
    e: any | undefined
  ) => void;
  onSuccess: (callback: (body: any) => void) => ICall;
  onError: (
    callback: (
      code: number | undefined,
      message: string | undefined,
      e: any | undefined
    ) => void
  ) => ICall;
  fetch: () => void;
}
class Call implements ICall {
  url = "";
  method: "POST" | "GET" | "DELETE" | "PUT" = "GET";
  data = {};

  constructor(
    url: string,
    method: "POST" | "GET" | "DELETE" | "PUT",
    data: any
  ) {
    this.url = url;
    this.method = method;
    this.data = data;
  }

  callbackOnSuccess = (body: any) => {};
  callbackOnError = (
    code: number | undefined,
    message: string | undefined,
    e: any | undefined
  ) => {};

  onSuccess = (callback: (body: any) => void) => {
    this.callbackOnSuccess = callback;
    return this;
  };

  onError = (
    callback: (
      code: number | undefined,
      message: string | undefined,
      e: any | undefined
    ) => void
  ) => {
    this.callbackOnError = callback;
    return this;
  };

  fetch = () => {
    requestManager.request(this.url, this.method, this.data).then(
      (body) => {
        this.callbackOnSuccess(body);
      },
      (e) => {
        if (e instanceof NotSuccessResponseError) {
          this.callbackOnError(e.httpCode, e.body, undefined);
        } else if (
          e instanceof Error &&
          e.message.startsWith("RequestManager.request: ")
        ) {
          const code = parseInt(e.message.split(" ")[1]);
          const message = e.message.split(" ").slice(2).join(" ");
          this.callbackOnError(code, message, undefined);
        } else {
          this.callbackOnError(undefined, undefined, e);
        }
      }
    );
  };
}

const requestManager = new RequestManager();

const httpPost = (url: string, data: any = undefined) => {
  return new Call(url, "POST", data) as ICall;
};

const httpGet = (url: string) => {
  return new Call(url, "GET", undefined) as ICall;
};

const httpDel = (url: string) => {
  return new Call(url, "DELETE", undefined) as ICall;
};

export type { ICall };
export { httpPost, httpGet, httpDel };
