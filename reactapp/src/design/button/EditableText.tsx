import { Input } from "@chakra-ui/react";
import { useEffect, useState } from "react";

export const EditableValue = ({
  value,
  onValueChange,
}: {
  value: any;
  onValueChange: (value: any) => void;
}) => {
  const [inputValue, setInputValue] = useState(value);

  // 부모로부터 value가 변경되면 inputValue도 업데이트
  useEffect(() => {
    setInputValue(value);
  }, [value]);

  // 입력 필드의 값이 변경될 때 호출
  const handleChange = (e: any) => {
    setInputValue(e.target.value);
  };

  // Enter 키를 누르면 콜백 호출
  const handleKeyDown = (e: any) => {
    if (e.key === "Enter") {
      const parsedValue = parseFloat(inputValue);
      if (!isNaN(parsedValue)) {
        onValueChange(parsedValue);
      } else {
        // 숫자가 아닌 경우 처리 (예: 알림 표시)
        alert("유효한 숫자를 입력해주세요.");
      }
    }
  };

  return (
    <Input
      type="text"
      value={inputValue}
      onChange={handleChange}
      onKeyDown={handleKeyDown}
      style={{
        padding: "8px",
        fontSize: "16px",
        border: "1px solid #ccc",
        borderRadius: "4px",
        width: "6rem",
      }}
    />
  );
};
