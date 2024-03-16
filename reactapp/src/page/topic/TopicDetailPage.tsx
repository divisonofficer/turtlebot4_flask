import { observer } from "mobx-react";
import { PageRoot } from "../../design/other/flexs";
import { TopicInfo } from "./TopicInfo";
import { topicStore } from "../../stores/TopicStore";

export const TopicDetailPage = observer(() => {
  return (
    <PageRoot
      title="Topic Detail"
      backPage={{
        title: "Topics",
        path: "/topic",
      }}
    >
      <TopicInfo topic={topicStore.getTopic(topicStore.topicDetailView)} />
    </PageRoot>
  );
});
